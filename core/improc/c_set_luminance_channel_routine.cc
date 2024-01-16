/*
 * c_set_luminance_channel_routine.cc
 *
 *  Created on: May 4, 2023
 *      Author: amyznikov
 */

#include "c_set_luminance_channel_routine.h"
#include <core/proc/unsharp_mask.h>
#include <core/ssprintf.h>


template<>
const c_enum_member * members_of<c_set_luminance_channel_routine::Colorspace>()
{
  static const c_enum_member members[] = {
      { c_set_luminance_channel_routine::Colorspace_Lab, "Lab", ""},
      { c_set_luminance_channel_routine::Colorspace_Luv, "Luv", ""},
      { c_set_luminance_channel_routine::Colorspace_HSV, "HSV", ""},
      { c_set_luminance_channel_routine::Colorspace_HLS, "HLS", ""},
      { c_set_luminance_channel_routine::Colorspace_YCrCb, "YCrCb", ""},
      {c_set_luminance_channel_routine::Colorspace_Lab}
  };

  return members;
}

void c_set_luminance_channel_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, luminance_channel,
      "Select which channel must become luminance (brightness)\n"
      "Normally it should be modt detailed channel ([infra] red for moon/planetary imaging)");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, colorspace,
      "Operation colorspace\n"
      "For non-linear color spaces like Lab/Luv the input image must be normalized to standard range");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, usharp_sigma,
      "Unsharp mask sigma\n"
      "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, usharp_alpha,
      "Unsharp mask alpha\n"
      "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, usharp_outmin,
      "Unsharp mask outmin\n"
      "");

  ADD_IMAGE_PROCESSOR_CTRL(ctls, usharp_outmax,
      "Unsharp mask outmax\n"
      "");
}

bool c_set_luminance_channel_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, luminance_channel);
    SERIALIZE_PROPERTY(settings, save, *this, colorspace);
    SERIALIZE_PROPERTY(settings, save, *this, usharp_sigma);
    SERIALIZE_PROPERTY(settings, save, *this, usharp_alpha);
    SERIALIZE_PROPERTY(settings, save, *this, usharp_outmin);
    SERIALIZE_PROPERTY(settings, save, *this, usharp_outmax);
    return true;
  }
  return false;
}

bool c_set_luminance_channel_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.channels() == 1 ) {
    if( usharp_sigma_ > 0 && usharp_alpha_ > 0 ) {
      unsharp_mask(image, mask, image, usharp_sigma_, usharp_alpha_,
          usharp_outmin_, usharp_outmax_);
    }
    return true;
  }

  if( image.channels() != 3 ) {
    return true; // silently ignore
  }

  cv::Mat luminance;

  if( !extract_channel(image, luminance, cv::noArray(), cv::noArray(), luminance_channel_) ) {
    CF_ERROR("extract_channel('%s') fails", toString(luminance_channel_));
    return false;
  }

  if( usharp_sigma_ > 0 && usharp_alpha_ > 0 ) {
    unsharp_mask(luminance, mask, luminance, usharp_sigma_, usharp_alpha_,
        usharp_outmin_, usharp_outmax_);
  }

  switch (colorspace_) {
    case Colorspace_Lab:

      cv::cvtColor(image,  image, cv::COLOR_BGR2Lab);
      if ( image.depth() >= CV_32F ) {
        cv::multiply(luminance, 100, luminance);
      }
      cv::insertChannel(luminance, image, 0);
      cv::cvtColor(image, image, cv::COLOR_Lab2BGR);
      break;

    case Colorspace_Luv:
      cv::cvtColor(image,  image, cv::COLOR_BGR2Luv);
      if ( image.depth() >= CV_32F ) {
        cv::multiply(luminance, 100, luminance);
      }
      cv::insertChannel(luminance, image, 0);
      cv::cvtColor(image, image, cv::COLOR_Luv2BGR);
      break;

    case Colorspace_HSV:
      cv::cvtColor(image,  image, cv::COLOR_BGR2HSV_FULL);
      cv::insertChannel(luminance, image, 2);
      cv::cvtColor(image,  image, cv::COLOR_HSV2BGR_FULL);
      break;

    case Colorspace_HLS:
      cv::cvtColor(image,  image, cv::COLOR_BGR2HLS_FULL);
      cv::insertChannel(luminance, image, 1);
      cv::cvtColor(image,  image, cv::COLOR_HLS2BGR_FULL);
      break;

    case Colorspace_YCrCb:
      cv::cvtColor(image,  image, cv::COLOR_BGR2YCrCb);
      cv::insertChannel(luminance, image, 0);
      cv::cvtColor(image,  image, cv::COLOR_YCrCb2BGR);
      break;

    default:
      CF_ERROR("APP BUG: unsupported colorspace %d ('%s') requested", colorspace_, toString(colorspace_));
      return false;
  }

  return true;
}

