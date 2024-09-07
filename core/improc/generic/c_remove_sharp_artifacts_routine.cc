/*
 * c_remove_sharp_artifacts_routine.cc
 *
 *  Created on: Jul 30, 2022
 *      Author: amyznikov
 */

#include "c_remove_sharp_artifacts_routine.h"
#include <core/proc/geo-reconstruction.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/reduce_channels.h>


template<class T>
static void wadd_(cv::InputOutputArray image, cv::InputArray _blured_image, const cv::Mat1f & w)
{
  cv::Mat_<T> img =
      image.getMat();

  const cv::Mat_<T> & blured =
      _blured_image.getMat();


  const int rows =
      image.rows();

  const int cols =
      image.cols();

  const int channels =
      image.channels();

  for( int y = 0; y < rows; ++y ) {

    T * imgp = img[y];
    const T * blurp = blured[y];

    const float *wp = w[y];

    for( int x = 0; x < cols; ++x ) {

      const double alpha =
          wp[x];

      if( alpha > FLT_EPSILON ) {
        for( int c = 0; c < channels; ++c ) {
          imgp[x * channels + c] = cv::saturate_cast<T>(
              alpha * imgp[x * channels + c] + (1 - alpha) * blurp[x * channels + c]);
        }
      }

    }
  }
}

static void wadd(cv::InputOutputArray image, cv::InputArray blured_image, const cv::Mat1f & w)
{
  switch (image.depth()) {
  case CV_8U:
    wadd_<uint8_t>(image, blured_image, w);
    break;
  case CV_8S:
    wadd_<int8_t>(image, blured_image, w);
    break;
  case CV_16U:
    wadd_<uint16_t>(image, blured_image, w);
    break;
  case CV_16S:
    wadd_<int16_t>(image, blured_image, w);
    break;
  case CV_32S:
    wadd_<int32_t>(image, blured_image, w);
    break;
  case CV_32F:
    wadd_<float>(image, blured_image, w);
    break;
  case CV_64F:
    wadd_<double>(image, blured_image, w);
    break;
  }
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

void c_remove_sharp_artifacts_routine::set_fill_holes(bool v)
{
  fill_holes_ = v;
}

bool c_remove_sharp_artifacts_routine::fill_holes() const
{
  return fill_holes_;
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


bool c_remove_sharp_artifacts_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {

    cv::Mat component_mask;

    bool planetary_disk_detected =
        simple_planetary_disk_detector(image, mask,
            1,
            0.25,
            2,
            nullptr,
            nullptr,
            &component_mask,
            nullptr/* &geometrical_center_*/,
            nullptr/* &debug_image*/);

    if( planetary_disk_detected ) {

    }
    else {

      const int cn =
          image.channels();

      const cv::Scalar s =
          estimate_noise(image, cv::noArray(), mask);

      double noise_value = 0;
      for( int c = 0; c < cn; ++c ) {
        noise_value += s[c];
      }

      noise_value /= cn;

      cv::Mat w;

      cv::compare(image, noise_scale_ * noise_value, w, cv::CMP_GT);
      if( cn > 1 ) {
        reduce_color_channels(w, cv::REDUCE_MAX);
      }

      component_mask = w;
    }

    if( fill_holes_ ) {
      geo_fill_holes(component_mask, component_mask, 8);
    }

    if ( erode_radius_.width > 0 || erode_radius_.height > 0 ) {
      cv::Mat1b se(2 * erode_radius_.height + 1, 2 * erode_radius_.width + 1, 255);
      cv::erode(component_mask, component_mask, se);
    }

    if ( mask_blur_radius_ > 0 ) {
      cv::GaussianBlur(component_mask, component_mask, cv::Size(),
          mask_blur_radius_, mask_blur_radius_,
          cv::BORDER_DEFAULT);
    }


    if ( show_mask_ ) {
      component_mask.convertTo(image, CV_32F, 1. / 255);
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
        component_mask.convertTo(component_mask, CV_32F, 1. / 255);
        wadd(image, blured_image, component_mask);
      }
    }

  }

  return true;
}

