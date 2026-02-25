/*
 * c_desaturate_edges_routine.cc
 *
 *  Created on: Oct 23, 2022
 *      Author: amyznikov
 */

#include "c_desaturate_edges_routine.h"
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/debug.h>

bool c_desaturate_edges_routine::compute_planetary_disk_weights(const cv::Mat & src_image, const cv::Mat & src_mask,
    cv::Mat1f & dst_image) const
{
  cv::Mat planetary_disk_mask;

  bool fOk =
      simple_planetary_disk_detector(src_image, src_mask,
          _gbsigma,
          _stdev_factor,
          _se_close_radius,
          nullptr,
          nullptr,
          &planetary_disk_mask);

  if( !fOk ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    return false;
  }


  cv::distanceTransform(planetary_disk_mask, planetary_disk_mask,
      cv::DIST_L2,
      cv::DIST_MASK_PRECISE,
      CV_32F);

  if ( _blur_radius > 0 ) {
    cv::GaussianBlur(planetary_disk_mask, planetary_disk_mask,
        cv::Size(),
        _blur_radius);
  }

  double min, max;
  cv::minMaxLoc(planetary_disk_mask,
      &min, &max);

  cv::multiply(planetary_disk_mask, 1. / max,
      planetary_disk_mask);

  if ( _l1norm ) {
    cv::sqrt(planetary_disk_mask,
        planetary_disk_mask);
  }

  dst_image =
      std::move(planetary_disk_mask);

  return true;
}

template<class T>
static void combine_images(const cv::Mat_<cv::Vec<T, 3>> & color_image, const cv::Mat_<cv::Vec<T, 3>> & gray_image,
    cv::Mat_<cv::Vec<T, 3>> & dst_image,
    const cv::Mat1f & weights, double alpha )
{

  dst_image.create(color_image.size());

  for ( int y = 0; y < color_image.rows; ++y ) {
    for ( int x = 0; x < color_image.cols; ++x ) {

      if ( !(weights[y][x] > 0) ) {
        dst_image[y][x] = color_image[y][x];
      }
      else {

        const double w1 = std::min(1., alpha + weights[y][x]);
        const double w2 = 1 - w1;

        for ( int i = 0; i < 3; ++i ) {

          dst_image[y][x][i] =
              cv::saturate_cast<T>( w1 * color_image[y][x][i] + w2 * gray_image[y][x][i]);
        }


      }
    }
  }
}

void c_desaturate_edges_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "alpha", ctx(&this_class::_alpha), "");
   ctlbind(ctls, "gbsigma", ctx(&this_class::_gbsigma), "");
   ctlbind(ctls, "stdev_factor", ctx(&this_class::_stdev_factor), "");
   ctlbind(ctls, "blur_radius", ctx(&this_class::_blur_radius), "");
   ctlbind(ctls, "se_close_radius", ctx(&this_class::_se_close_radius), "");
   ctlbind(ctls, "show_weights", ctx(&this_class::_show_weights), "");
   ctlbind(ctls, "l1norm", ctx(&this_class::_l1norm), "");
}

bool c_desaturate_edges_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _alpha);
    SERIALIZE_OPTION(settings, save, *this, _gbsigma);
    SERIALIZE_OPTION(settings, save, *this, _stdev_factor);
    SERIALIZE_OPTION(settings, save, *this, _blur_radius);
    SERIALIZE_OPTION(settings, save, *this, _l1norm);
    SERIALIZE_OPTION(settings, save, *this, _show_weights);
    return true;
  }
  return false;
}

bool c_desaturate_edges_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.channels() == 3 ) {

    cv::Mat gray_image;
    cv::Mat1f weights;

    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    if( compute_planetary_disk_weights(gray_image, mask.getMat(), weights) ) {

      if ( _show_weights ) {
        weights.copyTo(image);
      }
      else {

        cv::cvtColor(gray_image, gray_image, cv::COLOR_GRAY2BGR);

        switch (image.depth()) {

        case CV_8U: {
          cv::Mat3b combined_image;
          combine_images(cv::Mat3b(image.getMat()), cv::Mat3b(gray_image), combined_image, weights, _alpha);
          combined_image.copyTo(image);
          break;
        }

        case CV_8S: {
          typedef cv::Mat_<cv::Vec<int8_t, 3>> Mat3h;
          Mat3h combined_image;
          combine_images(Mat3h(image.getMat()), Mat3h(gray_image), combined_image, weights, _alpha);
          combined_image.copyTo(image);
          break;
        }

        case CV_16U: {
          cv::Mat3w combined_image;
          combine_images(cv::Mat3w(image.getMat()), cv::Mat3w(gray_image), combined_image, weights, _alpha);
          combined_image.copyTo(image);
          break;
        }

        case CV_16S: {
          cv::Mat3s combined_image;
          combine_images(cv::Mat3s(image.getMat()), cv::Mat3s(gray_image), combined_image, weights, _alpha);
          combined_image.copyTo(image);
          break;
        }

        case CV_32S: {
          cv::Mat3i combined_image;
          combine_images(cv::Mat3i(image.getMat()), cv::Mat3i(gray_image), combined_image, weights, _alpha);
          combined_image.copyTo(image);
          break;
        }

        case CV_32F: {
          cv::Mat3f combined_image;
          combine_images(cv::Mat3f(image.getMat()), cv::Mat3f(gray_image), combined_image, weights, _alpha);
          combined_image.copyTo(image);
          break;
        }

        case CV_64F: {
          cv::Mat3d combined_image;
          combine_images(cv::Mat3d(image.getMat()), cv::Mat3d(gray_image), combined_image, weights, _alpha);
          combined_image.copyTo(image);
          break;
        }
        }
      }
    }
  }

  return true;
}
