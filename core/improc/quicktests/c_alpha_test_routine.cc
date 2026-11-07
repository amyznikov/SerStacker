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
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/proc/divide.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>
#include <core/proc/divide.h>
#include <core/proc/multiply.h>


//template<>
//const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
//{
//  static const c_enum_member members[] = {
//      { c_alpha_test_routine::DISPLAY_SRC1, "SRC1", "" },
//      { c_alpha_test_routine::DISPLAY_SRC2, "SRC2", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC1, "SPEC1", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC2, "SPEC2", "" },
//      { c_alpha_test_routine::DISPLAY_SPEC3, "SPEC3", "" },
//      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13_DIFF, "SPEC13_DIFF", "" },
//      { c_alpha_test_routine::DISPLAY_CROSS_SPEC13, "CROSS_SPEC13", "" },
//      { c_alpha_test_routine::DISPLAY_CORR_MAP13, "CORR_MAP13", "" },
//      { c_alpha_test_routine::DISPLAY_SRC2}
//  };
//  return members;
//}

/////////////////////////////////////
namespace {
static cv::Size dscaleSize(cv::Size srcSize, int level)
{
  for( int l = 0; l < level; ++l ) {
    const cv::Size nextSize = cv::Size((srcSize.width + 1) / 2, (srcSize.height + 1) / 2);
    if( std::min(nextSize.width, nextSize.height) < 4 ) {
      break;
    }
    srcSize = nextSize;
  }
  return srcSize;
}


static bool pdownscale(cv::InputArray src, cv::Mat & dst, int level,
    int border_mode = cv::BORDER_DEFAULT)
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

    for( int i = sizes.size() - 1; i > 0; --i ) {
      cv::pyrUp(image, image, sizes[i]);
    }

    cv::resize(image, image, sizes[0], 0, 0, cv::INTER_LINEAR);
  }

  return true;
}

}

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, sigma);
    SERIALIZE_OPTION(settings, save, *this, eps);
    SERIALIZE_OPTION(settings, save, *this, kradius);
    SERIALIZE_OPTION(settings, save, *this, dscale);
    SERIALIZE_OPTION(settings, save, *this, uscale);
    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "sigma", CTL_CONTEXT(ctx, sigma), "");
  ctlbind(ctls, "eps", CTL_CONTEXT(ctx, eps), "");
  ctlbind(ctls, "kradius", CTL_CONTEXT(ctx, kradius), "");
  ctlbind(ctls, "dscale", CTL_CONTEXT(ctx, dscale), "");
  ctlbind(ctls, "uscale", CTL_CONTEXT(ctx, uscale), "");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
//  CF_DEBUG("enter");

  cv::Mat M1, M2, G1, G2, G;
  if ( image.channels() == 1 ) {
    image.getMat().convertTo(M1, CV_32F);
  }
  else {
    cv::cvtColor(image, M1, cv::COLOR_BGR2GRAY);
    M1.convertTo(M1, CV_32F);
  }


  if( dscale > 0 ) {
    pdownscale(M1, M1, dscale);
  }

  const int ksize1 = 2 * std::max(1, kradius) + 1;
//  const int ksize2 = 2 * ksize1 + 1;
  static thread_local cv::Mat1b SE1;
//  static thread_local cv::Mat1b SE2;
  if ( SE1.rows != ksize1 ) {
    SE1 = cv::Mat1b(ksize1, ksize1, 255);
  }
//  if ( SE2.rows != ksize2 ) {
//    SE2 = cv::Mat1b(ksize2, ksize2, 255);
//  }

  cv::morphologyEx(M1, G, cv::MORPH_GRADIENT, SE1, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//  cv::morphologyEx(M1, G2, cv::MORPH_GRADIENT, SE2, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//  cv::absdiff(G1, G2, G);
//  const float G1_mean = cv::norm(G1, cv::NORM_L1) / G1.size().area();
//  cv::add(G2, eps * G1_mean, G1);
//  divideImages(G1, G2, G);

  if( uscale > dscale ) {
    pdownscale(G, G, uscale - dscale);
    //cv::resize(G, G, dscaleSize(G.size(), uscale), 0, 0, cv::INTER_AREA);
  }

  if ( G.size() != image.size() ) {
    pupscale(G, image.size());
  }

  image.assign(G);

//  CF_DEBUG("leave");
  return true;
}


//  const float G1_mean = cv::norm(G1, cv::NORM_L1) / G1.size().area();
//  cv::morphologyEx(M2, G2, cv::MORPH_GRADIENT, SE, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
//  cv::add(G2, eps, G2);
//  divideImages(G1, G2, G);
