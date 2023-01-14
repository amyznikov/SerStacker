/*
 * c_remove_sharp_artifacts_routine.cc
 *
 *  Created on: Jul 30, 2022
 *      Author: amyznikov
 */

#include "c_remove_sharp_artifacts_routine.h"
#include <core/proc/estimate_noise.h>
#include <core/proc/reduce_channels.h>

c_remove_sharp_artifacts_routine::c_class_factory c_remove_sharp_artifacts_routine::class_factory;


template<class T>
static void wadd_(cv::InputOutputArray image, cv::InputArray _src, const cv::Mat1f & w)
{
  cv::Mat_<T> dst =
      image.getMatRef();

  const cv::Mat_<T> & src =
      _src.getMat();

  const int cn =
      image.channels();

  for( int y = 0; y < dst.rows; ++y ) {

    T *dp = dst[y];
    const T *sp = src[y];
    const float *wp = w[y];

    for( int x = 0; x < dst.cols; ++x ) {

      const double alpha = wp[x / cn];

      dp[x] = cv::saturate_cast<T>(
          alpha * dp[x] + (1 - alpha) * sp[x]);

    }
  }
}

static void wadd(cv::InputOutputArray image, cv::InputArray src, const cv::Mat1f & w)
{
  switch (image.depth()) {
  case CV_8U:
    wadd_<uint8_t>(image, src, w);
    break;
  case CV_8S:
    wadd_<int8_t>(image, src, w);
    break;
  case CV_16U:
    wadd_<uint16_t>(image, src, w);
    break;
  case CV_16S:
    wadd_<int16_t>(image, src, w);
    break;
  case CV_32S:
    wadd_<int32_t>(image, src, w);
    break;
  case CV_32F:
    wadd_<float>(image, src, w);
    break;
  case CV_64F:
    wadd_<double>(image, src, w);
    break;
  }
}

c_remove_sharp_artifacts_routine::c_remove_sharp_artifacts_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_remove_sharp_artifacts_routine::ptr c_remove_sharp_artifacts_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

void c_remove_sharp_artifacts_routine::set_erode_radius(const cv::Size & v)
{
  erode_radius_ = v;
}

const cv::Size & c_remove_sharp_artifacts_routine::erode_radius() const
{
  return erode_radius_;
}


void c_remove_sharp_artifacts_routine::set_mask_blur_radius(double v)
{
  mask_blur_radius_ = v;
}

double c_remove_sharp_artifacts_routine::mask_blur_radius() const
{
  return mask_blur_radius_;
}

void c_remove_sharp_artifacts_routine::set_edge_blur_radius(double v)
{
  edge_blur_radius_ = v;
}

double c_remove_sharp_artifacts_routine::edge_blur_radius() const
{
  return edge_blur_radius_;
}

void c_remove_sharp_artifacts_routine::set_noise_scale(double v)
{
  noise_scale_ = v;
}

double c_remove_sharp_artifacts_routine::noise_scale() const
{
  return noise_scale_;
}

void c_remove_sharp_artifacts_routine::set_show_mask(bool v)
{
  show_mask_ = v;
}

bool c_remove_sharp_artifacts_routine::show_mask() const
{
  return show_mask_;
}

void c_remove_sharp_artifacts_routine::set_show_blured_image(bool v)
{
  show_blured_image_ = v;
}

bool c_remove_sharp_artifacts_routine::show_blured_image() const
{
  return show_blured_image_;
}

bool c_remove_sharp_artifacts_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, *this, erode_radius);
  LOAD_PROPERTY(settings, *this, noise_scale);
  LOAD_PROPERTY(settings, *this, mask_blur_radius);
  LOAD_PROPERTY(settings, *this, edge_blur_radius);
  LOAD_PROPERTY(settings, *this, show_mask);
  LOAD_PROPERTY(settings, *this, show_blured_image);

  return true;
}

bool c_remove_sharp_artifacts_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, erode_radius);
  SAVE_PROPERTY(settings, *this, noise_scale);
  SAVE_PROPERTY(settings, *this, mask_blur_radius);
  SAVE_PROPERTY(settings, *this, edge_blur_radius);
  SAVE_PROPERTY(settings, *this, show_mask);
  SAVE_PROPERTY(settings, *this, show_blured_image);

  return true;
}

bool c_remove_sharp_artifacts_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {


    const int cn =
        image.channels();

    const cv::Scalar s =
        estimate_noise(image, cv::noArray(), mask);

    double noise_value = 0;
    for ( int c = 0; c < cn;  ++c ) {
      noise_value += s[c];
    }

    noise_value /= cn;

    // CF_DEBUG("c_remove_sharp_artifacts_routine: noise_value=%g", noise_value);

    cv::Mat w;
    cv::compare(image, noise_scale_ * noise_value, w, cv::CMP_GT);
    if ( cn > 1 ) {
      reduce_color_channels(w, cv::REDUCE_MAX);
    }

    if ( erode_radius_.width > 0 || erode_radius_.height > 0 ) {
      cv::Mat1b se(2 * erode_radius_.height + 1, 2 * erode_radius_.width + 1, 255);
      cv::erode(w, w, se);
    }

    if ( mask_blur_radius_ > 0 ) {
      cv::GaussianBlur(w, w, cv::Size(),
          mask_blur_radius_, mask_blur_radius_,
          cv::BORDER_DEFAULT);
    }


    if ( show_mask_ ) {
      w.convertTo(image, CV_32F, 1. / 255);
    }
    else {

      cv::Mat blured_image;

      cv::medianBlur(image, blured_image, 5);

      if( edge_blur_radius_ > 0 ) {
        cv::GaussianBlur(blured_image, blured_image, cv::Size(),
            edge_blur_radius_, edge_blur_radius_);
      }

      if ( show_blured_image_ ) {
        blured_image.copyTo(image);
      }
      else {
        w.convertTo(w, CV_32F, 1. / 255);
        wadd(image, blured_image, w);
      }
    }

  }

  return true;
}

