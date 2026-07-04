/*
 * c_desaturate_edges_routine.cc
 *
 *  Created on: Oct 23, 2022
 *      Author: amyznikov
 */

#include "c_desaturate_edges_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/pixtype.h>
#include <core/debug.h>


template<class T>
static bool combine_images(cv::InputArray _color_image, cv::InputArray _gray_color_image, cv::OutputArray _dst,
    const cv::Mat1f & weights, double alpha)
{
  const cv::Mat_<cv::Vec<T, 3>> color_image = _color_image.getMat();
  const cv::Mat_<cv::Vec<T, 3>> gray_image = _gray_color_image.getMat();
  cv::Mat_<cv::Vec<T, 3>> combined_image(color_image.size()); //  = _dst.getMatRef();

  cv::parallel_for_(cv::Range(0, color_image.rows), [&, alpha](const auto & range) {
    const int xmax = color_image.cols;
    for ( int y = range.start; y < range.end; ++y ) {

      const auto * srcp1 = color_image[y];
      const auto * srcp2 = gray_image[y];
      const auto * wp = weights[y];
      auto * __restrict dstp = combined_image[y];

      for ( int x = 0; x < xmax; ++x ) {
        if ( !(wp[x] > 0) ) {
          dstp[x] = srcp1[x];
        }
        else {
          const double w1 = std::min(1., alpha + wp[x]);
          const double w2 = 1 - w1;
          for ( int i = 0; i < 3; ++i ) {
            dstp[x][i] = cv::saturate_cast<T>( w1 * srcp1[x][i] + w2 * srcp2[x][i]);
          }
        }
      }
    }
  });

  _dst.move(combined_image);

  return true;
}

void c_desaturate_edges_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "alpha", ctx(&this_class::_alpha), "");
   ctlbind(ctls, "gsigma", ctx(&this_class::_gsigma), "");
   ctlbind(ctls, "se_radius", ctx(&this_class::_se_radius), "");
   ctlbind(ctls, "blur_radius", ctx(&this_class::_blur_radius), "");
   ctlbind(ctls, "show_weights", ctx(&this_class::_show_weights), "");
   ctlbind(ctls, "show_mask", ctx(&this_class::_show_mask), "");
   ctlbind(ctls, "l1norm", ctx(&this_class::_l1norm), "");
}

bool c_desaturate_edges_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _alpha);
    SERIALIZE_OPTION(settings, save, *this, _gsigma);
    SERIALIZE_OPTION(settings, save, *this, _blur_radius);
    SERIALIZE_OPTION(settings, save, *this, _show_mask);
    SERIALIZE_OPTION(settings, save, *this, _show_weights);
    SERIALIZE_OPTION(settings, save, *this, _l1norm);
    return true;
  }
  return false;
}

bool c_desaturate_edges_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.channels() != 3 ) {
    return true;
  }

  cv::Mat gray_image;
  cv::Mat planetary_disk_mask;
  cv::Mat1f weights;

  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  const bool planetaryDiskDetected =
      simple_planetary_disk_detector(image, mask,
          _gsigma,
          _se_radius,
          nullptr,
          nullptr,
          &planetary_disk_mask);

  if ( !planetaryDiskDetected ) {
    CF_ERROR("simple_planetary_disk_detector fails");
    return false;
  }

  if ( _show_mask ) {
    image.move(planetary_disk_mask);
    return true;
  }

  cv::distanceTransform(planetary_disk_mask, weights,
      cv::DIST_L2,
      cv::DIST_MASK_PRECISE,
      CV_32F);

  if( _blur_radius > 0 ) {
    cv::GaussianBlur(weights, weights, cv::Size(), _blur_radius, _blur_radius);
  }

  double min, max;
  cv::minMaxLoc(weights, &min, &max);
  cv::multiply(weights, 1. / max, weights);
  if( _l1norm ) {
    cv::sqrt(weights, weights);
  }

  if ( _show_weights ) {
    image.move(weights);
    return true;
  }

  cv::cvtColor(gray_image, gray_image, cv::COLOR_GRAY2BGR);
  CV_DISPATCH(image.depth(), combine_images, image, gray_image, image, weights, _alpha);


//  switch (image.depth()) {
//
//    case CV_8U: {
//      cv::Mat3b combined_image;
//      combine_images(cv::Mat3b(image.getMat()), cv::Mat3b(gray_image), combined_image, weights, _alpha);
//      image.move(combined_image);
//      break;
//    }
//
//    case CV_8S: {
//      typedef cv::Mat_<cv::Vec<int8_t, 3>> Mat3h;
//      Mat3h combined_image;
//      combine_images(Mat3h(image.getMat()), Mat3h(gray_image), combined_image, weights, _alpha);
//      image.move(combined_image);
//      break;
//    }
//
//    case CV_16U: {
//      cv::Mat3w combined_image;
//      combine_images(cv::Mat3w(image.getMat()), cv::Mat3w(gray_image), combined_image, weights, _alpha);
//      image.move(combined_image);
//      break;
//    }
//
//    case CV_16S: {
//      cv::Mat3s combined_image;
//      combine_images(cv::Mat3s(image.getMat()), cv::Mat3s(gray_image), combined_image, weights, _alpha);
//      image.move(combined_image);
//      break;
//    }
//
//    case CV_32S: {
//      cv::Mat3i combined_image;
//      combine_images(cv::Mat3i(image.getMat()), cv::Mat3i(gray_image), combined_image, weights, _alpha);
//      image.move(combined_image);
//      break;
//    }
//
//    case CV_32F: {
//      cv::Mat3f combined_image;
//      combine_images(cv::Mat3f(image.getMat()), cv::Mat3f(gray_image), combined_image, weights, _alpha);
//      image.move(combined_image);
//      break;
//    }
//
//    case CV_64F: {
//      cv::Mat3d combined_image;
//      combine_images(cv::Mat3d(image.getMat()), cv::Mat3d(gray_image), combined_image, weights, _alpha);
//      image.move(combined_image);
//      break;
//    }
//  }


  return true;
}
