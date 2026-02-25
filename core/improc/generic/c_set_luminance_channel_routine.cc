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

void c_set_luminance_channel_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "luminance_channel", ctx(&this_class::_luminance_channel), "");
   ctlbind(ctls, "colorspace", ctx(&this_class::_colorspace), "");
   ctlbind(ctls, "usharp_sigma", ctx(&this_class::_usharp_sigma), "");
   ctlbind(ctls, "usharp_alpha", ctx(&this_class::_usharp_alpha), "");
   ctlbind(ctls, "outmin", ctx(&this_class::_usharp_outmin), "");
   ctlbind(ctls, "outmax", ctx(&this_class::_usharp_outmax), "");
}

bool c_set_luminance_channel_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _luminance_channel);
    SERIALIZE_OPTION(settings, save, *this, _colorspace);
    SERIALIZE_OPTION(settings, save, *this, _usharp_sigma);
    SERIALIZE_OPTION(settings, save, *this, _usharp_alpha);
    SERIALIZE_OPTION(settings, save, *this, _usharp_outmin);
    SERIALIZE_OPTION(settings, save, *this, _usharp_outmax);
    return true;
  }
  return false;
}

bool c_set_luminance_channel_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.channels() == 1 ) {
    if( _usharp_sigma > 0 && _usharp_alpha > 0 ) {
      unsharp_mask(image, mask, image, _usharp_sigma, _usharp_alpha,
          _usharp_outmin, _usharp_outmax);
    }
    return true;
  }

  if( image.channels() != 3 ) {
    return true; // silently ignore
  }

  cv::Mat luminance;

  if( !extract_channel(image, luminance, cv::noArray(), cv::noArray(), _luminance_channel) ) {
    CF_ERROR("extract_channel('%s') fails", toString(_luminance_channel));
    return false;
  }

  if( _usharp_sigma > 0 && _usharp_alpha > 0 ) {
    unsharp_mask(luminance, mask, luminance, _usharp_sigma, _usharp_alpha,
        _usharp_outmin, _usharp_outmax);
  }

  switch (_colorspace) {
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
      CF_ERROR("APP BUG: unsupported colorspace %d ('%s') requested", _colorspace, toString(_colorspace));
      return false;
  }

  return true;
}

