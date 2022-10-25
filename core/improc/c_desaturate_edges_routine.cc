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

c_desaturate_edges_routine::c_class_factory c_desaturate_edges_routine::class_factory;

c_desaturate_edges_routine::c_desaturate_edges_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_desaturate_edges_routine::c_desaturate_edges_routine(double w, bool enabled)
  : base(&class_factory, enabled),
    alpha_(w)
{
}


c_desaturate_edges_routine::ptr c_desaturate_edges_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_desaturate_edges_routine::ptr c_desaturate_edges_routine::create(double w, bool enabled)
{
  return ptr(new this_class(w, enabled));
}


void c_desaturate_edges_routine::set_alpha(double v)
{
  alpha_ = v;
}

double c_desaturate_edges_routine::alpha() const
{
  return alpha_;
}

bool c_desaturate_edges_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  settings.get("alpha", &alpha_);

  return true;
}

bool c_desaturate_edges_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  settings.set("alpha", alpha_);

  return true;
}

bool c_desaturate_edges_routine::compute_planetary_disk_weights(const cv::Mat & src_image, const cv::Mat & src_mask,
    cv::Mat1f & dst_image) const
{
  cv::Mat planetary_disk_mask;

  bool fOk =
      simple_planetary_disk_detector(src_image, src_mask,
          nullptr,
          2,
          nullptr,
          &planetary_disk_mask);

  if( !fOk ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    return false;
  }

  cv::Scalar m, s;
  cv::meanStdDev(src_image, m, s, src_mask);
  cv::bitwise_and(planetary_disk_mask, src_image > s[0] / 4, planetary_disk_mask);
  morphological_smooth_close(planetary_disk_mask, planetary_disk_mask, cv::Mat1b(3, 3, 255));
  geo_fill_holes(planetary_disk_mask, planetary_disk_mask, 8);

  //src_image.copyTo(dst_image);

  double min, max;

  cv::distanceTransform(planetary_disk_mask, planetary_disk_mask, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
  cv::GaussianBlur(planetary_disk_mask, planetary_disk_mask, cv::Size(), 10);
  cv::minMaxLoc(planetary_disk_mask, &min, &max);
  //cv::multiply(planetary_disk_mask, planetary_disk_mask, planetary_disk_mask, 1. / (max * max));
  cv::multiply(planetary_disk_mask, 1. / max, planetary_disk_mask);
  dst_image = std::move(planetary_disk_mask);

  //planetary_disk_mask.copyTo(dst_image, planetary_disk_mask > FLT_EPSILON);

  return true;
}

template<class T>
static void combine_images(const cv::Mat_<cv::Vec<T, 3>> & color_image, const cv::Mat_<cv::Vec<T, 3>> & gray_image,
    cv::Mat_<cv::Vec<T, 3>> & dst_image,
    const cv::Mat1f & weigts, double alpha )
{

  dst_image.create(color_image.size());

  for ( int y = 0; y < color_image.rows; ++y ) {
    for ( int x = 0; x < color_image.cols; ++x ) {

      if ( !(weigts[y][x] > 0) ) {
        dst_image[y][x] = gray_image[y][x];
      }
      else {

        const double w1 = std::min(1., alpha + weigts[y][x]);
        const double w2 = 1 - w1;

        for ( int i = 0; i < 3; ++i ) {

          dst_image[y][x][i] =
              cv::saturate_cast<T>( w1 * color_image[y][x][i] + w2 * gray_image[y][x][i]);
        }


      }
    }
  }
}

bool c_desaturate_edges_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.channels() == 3 ) {

    cv::Mat gray_image;
    cv::Mat1f weights;

    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    if( compute_planetary_disk_weights(gray_image, mask.getMat(), weights) ) {

      cv::cvtColor(gray_image, gray_image, cv::COLOR_GRAY2BGR);

      switch (image.depth()) {

      case CV_8U: {
        cv::Mat3b combined_image;
        combine_images(cv::Mat3b(image.getMat()), cv::Mat3b(gray_image), combined_image, weights, alpha_);
        combined_image.copyTo(image);
        break;
      }

      case CV_8S: {
        typedef cv::Mat_<cv::Vec<int8_t, 3>> Mat3h;
        Mat3h combined_image;
        combine_images(Mat3h(image.getMat()), Mat3h(gray_image), combined_image, weights, alpha_);
        combined_image.copyTo(image);
        break;
      }

      case CV_16U: {
        cv::Mat3w combined_image;
        combine_images(cv::Mat3w(image.getMat()), cv::Mat3w(gray_image), combined_image, weights, alpha_);
        combined_image.copyTo(image);
        break;
      }

      case CV_16S: {
        cv::Mat3s combined_image;
        combine_images(cv::Mat3s(image.getMat()), cv::Mat3s(gray_image), combined_image, weights, alpha_);
        combined_image.copyTo(image);
        break;
      }

      case CV_32S: {
        cv::Mat3i combined_image;
        combine_images(cv::Mat3i(image.getMat()), cv::Mat3i(gray_image), combined_image, weights, alpha_);
        combined_image.copyTo(image);
        break;
      }

      case CV_32F: {
        cv::Mat3f combined_image;
        combine_images(cv::Mat3f(image.getMat()), cv::Mat3f(gray_image), combined_image, weights, alpha_);
        combined_image.copyTo(image);
        break;
      }

      case CV_64F: {
        cv::Mat3d combined_image;
        combine_images(cv::Mat3d(image.getMat()), cv::Mat3d(gray_image), combined_image, weights, alpha_);
        combined_image.copyTo(image);
        break;
      }
      }
    }
  }

  return true;
}
