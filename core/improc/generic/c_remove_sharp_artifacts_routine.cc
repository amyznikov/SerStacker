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

void c_remove_sharp_artifacts_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "erode_radius", ctx(&this_class::_erode_radius), "SE radius for erode");
   ctlbind(ctls, "mask_blur_radius", ctx(&this_class::_mask_blur_radius), "GaussianBlur sigma");
   ctlbind(ctls, "edge_blur_radius", ctx(&this_class::_edge_blur_radius), "GaussianBlur sigma");
   ctlbind(ctls, "fill_holes", ctx(&this_class::_fill_holes), "Fill holes inide mask (jovian satellite shadows etc)");
   ctlbind(ctls, "noise_scale", ctx(&this_class::_noise_scale), "noise scale");
   ctlbind(ctls, "show_mask", ctx(&this_class::_show_mask), "show objects mask instead of processing");
   ctlbind(ctls, "show_blured_image", ctx(&this_class::_show_blured_image), "show blured image instead of processing");
}

bool c_remove_sharp_artifacts_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _erode_radius);
    SERIALIZE_OPTION(settings, save, *this, _mask_blur_radius);
    SERIALIZE_OPTION(settings, save, *this, _edge_blur_radius);
    SERIALIZE_OPTION(settings, save, *this, _fill_holes);
    SERIALIZE_OPTION(settings, save, *this, _noise_scale);
    SERIALIZE_OPTION(settings, save, *this, _show_mask);
    SERIALIZE_OPTION(settings, save, *this, _show_blured_image);
    return true;
  }
  return false;
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

      cv::compare(image, _noise_scale * noise_value, w, cv::CMP_GT);
      if( cn > 1 ) {
        reduce_color_channels(w, cv::REDUCE_MAX);
      }

      component_mask = w;
    }

    if( _fill_holes ) {
      geo_fill_holes(component_mask, component_mask, 8);
    }

    if ( _erode_radius.width > 0 || _erode_radius.height > 0 ) {
      cv::Mat1b se(2 * _erode_radius.height + 1, 2 * _erode_radius.width + 1, 255);
      cv::erode(component_mask, component_mask, se);
    }

    if ( _mask_blur_radius > 0 ) {
      cv::GaussianBlur(component_mask, component_mask, cv::Size(),
          _mask_blur_radius, _mask_blur_radius,
          cv::BORDER_DEFAULT);
    }


    if ( _show_mask ) {
      component_mask.convertTo(image, CV_32F, 1. / 255);
    }
    else {
      cv::Mat blured_image;

      cv::medianBlur(image, blured_image, 5);

      if( _edge_blur_radius > 0 ) {
        cv::GaussianBlur(blured_image, blured_image, cv::Size(),
            _edge_blur_radius, _edge_blur_radius);
      }

      if ( _show_blured_image ) {
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

