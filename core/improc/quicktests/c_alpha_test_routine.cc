/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>


template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_SRC, "SRC", "" },
      { c_alpha_test_routine::DISPLAY_G0, "G0", "" },
//      { c_alpha_test_routine::DISPLAY_DIFF, "DIFF", "" },
//      { c_alpha_test_routine::DISPLAY_SUMM, "SUMM", "" },
//      { c_alpha_test_routine::DISPLAY_RATIO, "RATIO", "" },
      { c_alpha_test_routine::DISPLAY_SRC}
  };
  return members;
}


namespace {
static bool pdownscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_DEFAULT)
{
  if( std::min(src.cols(), src.rows()) < 4 ) {
    src.copyTo(dst);
  }
  else {

    cv::pyrDown(src, dst, cv::Size(), border_mode);

    if( std::min(dst.cols, dst.rows) >= 4 ) {

      for( int l = 1; l < level; ++l ) {

        cv::pyrDown(dst, dst, cv::Size(), border_mode);

        if( std::min(dst.cols, dst.rows) < 4 ) {
          break;
        }
      }
    }
  }

  return true;
}

static bool pupscale(cv::Mat & image, cv::Size dstSize)
{
  const cv::Size inputSize = image.size();

  if( inputSize != dstSize ) {

    std::vector<cv::Size> sizes;

    sizes.emplace_back(dstSize);

    while (42) {
      const cv::Size nextSize((sizes.back().width + 1) / 2, (sizes.back().height + 1) / 2);
      if( nextSize == inputSize ) {
        break;
      }
      if( nextSize.width < inputSize.width || nextSize.height < inputSize.height ) {
        CF_ERROR("FATAL: invalid next size : nextSize=%dx%d inputSize=%dx%d",
            nextSize.width, nextSize.height,
            inputSize.width, inputSize.height);
        return false;
      }
      sizes.emplace_back(nextSize);
    }

    for( int i = sizes.size() - 1; i >= 0; --i ) {
      cv::pyrUp(image, image, sizes[i]);
    }
  }

  return true;
}

//static inline void computeMorphGradient(cv::InputArray src, cv::OutputArray dst, cv::InputArray SE)
//{
//  cv::morphologyEx(src, dst, cv::MORPH_GRADIENT,
//      SE, cv::Point(-1, -1), 1,
//      cv::BORDER_REPLICATE);
//}

//static double computeMichelsonQuality(cv::InputArray src, cv::InputArray SE)
//{
//  cv::Mat imax, imin, num, den;
//
//  cv::dilate(src, imax, SE);
//  cv::erode(src, imin, SE);
//  cv::absdiff(imax, imin, num);
//  cv::add(imax, imin, den);
//
//  double total_gradient = cv::sum(num)[0];
//  double total_energy = cv::sum(den)[0];
//
//  return total_gradient / total_energy;
//}


} // namespace

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "display", CTL_CONTEXT(ctx, _display), "");
  ctlbind(ctls, "Intensity channel: ", CTL_CONTEXT(ctx, _intensity_channel), "Select intensity channel for spectrum analysis");
  ctlbind(ctls, "l0: ", CTL_CONTEXT(ctx, _l0), "");
  ctlbind(ctls, "l0: ", CTL_CONTEXT(ctx, _l2), "");
  ctlbind(ctls, "kradius0:", CTL_CONTEXT(ctx, _kradius0), "");
  ctlbind(ctls, "kradius1:", CTL_CONTEXT(ctx, _kradius1), "");
}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _intensity_channel);
    SERIALIZE_OPTION(settings, save, *this, _l0);
    SERIALIZE_OPTION(settings, save, *this, _l2);
    SERIALIZE_OPTION(settings, save, *this, _kradius0);
    SERIALIZE_OPTION(settings, save, *this, _kradius1);

    return true;
  }
  return false;
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
//  CF_DEBUG("c_alpha_test_routine: ENTER");

  cv::Mat src, m, g0;

  extract_channel(image, src, cv::noArray(), cv::noArray(), _intensity_channel, CV_32F, false);
  if( _l0 > 0 ) {
    pdownscale(src, src, _l0);
  }

  // kurtosis
  const cv::Size ksize0(2 * std::max(1, _kradius0) + 1, 2 * std::max(1, _kradius0) + 1);
  cv::boxFilter(src, m, CV_32F, ksize0, cv::Point(-1, -1), true, cv::BORDER_REPLICATE);
  cv::subtract(src, m, g0);
  cv::multiply(g0, g0, g0);

  const double W = cv::sum(g0)[0];
  cv::multiply(g0, g0, g0, 1. / W);

  const double Q = cv::sum(g0)[0] ;
  CF_DEBUG("Q = %g", Q);

  if ( _l2 > 0 ) {
    pdownscale(g0, g0, _l2);
  }

  switch (_display)
  {
    case DISPLAY_SRC: {
      if( src.size() != image.size() ) {
        pupscale(src, image.size());
      }
      image.move(src);
      break;
    }
    case DISPLAY_G0: {
      if( g0.size() != image.size() ) {
        pupscale(g0, image.size());
      }
      image.move(g0);
      break;
    }
  }

  return true;
}


//bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
//{
//  CF_DEBUG("c_alpha_test_routine: ENTER");
//
//  cv::Mat src, imax, imin, num, den, ratio;
//
//  extract_channel(image, src, cv::noArray(), cv::noArray(), _intensity_channel, CV_32F, false);
//  if( _display == DISPLAY_SRC ) {
//    image.move(src);
//    return true;
//  }
//
//  if ( _l0 > 0 ) {
//    pdownscale(src, src, _l0);
//  }
//
//  const cv::Size ksize0(2 * std::max(1, _kradius0) + 1, 2 * std::max(1, _kradius0) + 1);
//  const cv::Mat1b SE(ksize0, 255);
//
//  cv::dilate(src, imax, SE);
//  cv::erode(src, imin, SE);
//  cv::subtract(imax, imin, num);
//  cv::add(imax, imin, den);
//
//  if ( _l2 > 0 ) {
//    pdownscale(num, num, _l2);
//    pdownscale(den, den, _l2);
//  }
//
//  cv::divide(num, den, ratio);
//
//  switch (_display)
//  {
//    case DISPLAY_DIFF: {
//      if( num.size() != image.size() ) {
//        pupscale(num, image.size());
//      }
//      image.move(num);
//      break;
//    }
//    case DISPLAY_SUMM: {
//      if( den.size() != image.size() ) {
//        pupscale(den, image.size());
//      }
//      image.move(den);
//      break;
//    }
//    case DISPLAY_RATIO: {
//      if( ratio.size() != image.size() ) {
//        pupscale(ratio, image.size());
//      }
//      image.move(ratio);
//
//      break;
//    }
//  }
//
//  return true;
//}
//

//bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
//{
//  CF_DEBUG("c_alpha_test_routine: ENTER");
//
//  cv::Mat src, m0, m1, g0, g1;
//
//  const cv::Size ksize0(2 * std::max(1, _kradius0) + 1, 2 * std::max(1, _kradius0) + 1);
//  const cv::Size ksize1(2 * std::max(1, _kradius1) + 1, 2 * std::max(1, _kradius1) + 1);
//
//  extract_channel(image, src, cv::noArray(), cv::noArray(), _intensity_channel, CV_32F, false);
//  if ( _l0 > 0 ) {
//    pdownscale(src, src, _l0);
//  }
//
//  cv::boxFilter(src, m0, CV_32F, ksize0, cv::Point(-1, -1), true, cv::BORDER_REPLICATE);
//  cv::absdiff(src, m0, g0);
//
//  cv::boxFilter(src, m1, CV_32F, ksize1, cv::Point(-1, -1), true, cv::BORDER_REPLICATE);
//  cv::absdiff(src, m1, g1);
//
//  if ( _display != DISPLAY_SRC ){
//
//    switch (_display)
//    {
//      case DISPLAY_G0: {
//        // for the visualization only: restore original scale
//        if ( g0.size() != image.size() ) {
//          pupscale(g0, image.size());
//        }
//        image.move(g0);
//        break;
//      }
//      case DISPLAY_MORPH1: {
//        // for the visualization only: restore original scale
//        if ( g1.size() != image.size() ) {
//          pupscale(g1, image.size());
//        }
//        image.move(g1);
//        break;
//      }
//      case DISPLAY_MORPH_ABSDIFF: {
//        cv::Mat mid;
//        cv::absdiff(g0, g1, mid);
//        cv::boxFilter(mid, mid, CV_32F, ksize1, cv::Point(-1, -1), true, cv::BORDER_REPLICATE);
//        // for the visualization only: restore original scale
//        if ( mid.size() != image.size() ) {
//          pupscale(mid, image.size());
//        }
//        image.move(mid);
//        break;
//      }
//      case DISPLAY_MORPH_RATIO: {
//        cv::Mat num, den;
//        const cv::Size ksize_big(5 * std::max(1, _kradius1) + 1, 5 * std::max(1, _kradius1) + 1);
//
//        cv::multiply(g0, g1, num);
//        cv::boxFilter(num, num, CV_32F, ksize_big, cv::Point(-1, -1), true, cv::BORDER_REPLICATE);
//        cv::boxFilter(g1, den, CV_32F, ksize_big, cv::Point(-1, -1), true, cv::BORDER_REPLICATE);
//        cv::divide(g0, mid + 1e-6, ratio);
//
//        // for the visualization only: restore original scale
//        if ( ratio.size() != image.size() ) {
//          pupscale(ratio, image.size());
//        }
//        image.move(ratio);
//        break;
//      }
//    }
//  }
//
//  return true;
//}
