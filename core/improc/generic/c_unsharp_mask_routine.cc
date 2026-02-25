/*
 * c_unsharp_mask_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_unsharp_mask_routine.h"
#include <core/proc/unsharp_mask.h>
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_unsharp_mask_routine::COLOR_CHANNEL>()
{
  static const c_enum_member members[] = {
      { c_unsharp_mask_routine::COLOR_CHANNEL_ALL, "ALL", "aa" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_0, "0", "aa" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_1, "1", "aa" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_2, "2", "aa" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_3, "3", "aa" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_YCrCb, "Luminance (YCrCb)", "" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_Lab, "Luminance (Lab)", "" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_Luv, "Luminance (Luv)", "" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_HSV, "Luminance (HSV)", "" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_HLS, "Luminance (HLS)", "" },
      { c_unsharp_mask_routine::COLOR_CHANNEL_ALL },
  };

  return members;
}

void c_unsharp_mask_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "channel", ctx(&this_class::_channel), "Color channel for sharpening");
   ctlbind(ctls, "sigma", ctx(&this_class::_sigma), "");
   ctlbind(ctls, "alpha", ctx(&this_class::_alpha), "");
   ctlbind(ctls, "outmin", ctx(&this_class::_outmin), "");
   ctlbind(ctls, "outmax", ctx(&this_class::_outmax), "");
   ctlbind(ctls, "blur_color_channels", ctx(&this_class::_blur_color_channels), "Gaussian blur sigma for color channels");
}

bool c_unsharp_mask_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _channel);
    SERIALIZE_OPTION(settings, save, *this, _sigma);
    SERIALIZE_OPTION(settings, save, *this, _alpha);
    SERIALIZE_OPTION(settings, save, *this, _blur_color_channels);
    SERIALIZE_OPTION(settings, save, *this, _outmin);
    SERIALIZE_OPTION(settings, save, *this, _outmax);
    return true;
  }
  return false;
}

bool c_unsharp_mask_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _channel == COLOR_CHANNEL_ALL || image.channels() < 2 ) {
    return unsharp_mask(image, _ignore_mask ? cv::noArray() : mask, image, _sigma, _alpha, _outmin, _outmax);
  }

  cv::Mat tmp;
  int cc;
  double cscale = 1.0;

  switch (_channel) {

    case COLOR_CHANNEL_0:
      cc = 0;
      if( image.channels() < 1 ) {
        CF_ERROR("Channel 0 not exists in input image");
        return false;
      }
      break;

    case COLOR_CHANNEL_1:
      cc = 1;
      if( image.channels() < 2 ) {
        CF_ERROR("Channel 1 not exists in input image");
        return false;
      }
      break;

    case COLOR_CHANNEL_2:
      cc = 2;
      if( image.channels() < 3 ) {
        CF_ERROR("Channel 2 not exists in input image");
        return false;
      }
      break;

    case COLOR_CHANNEL_3:
      cc = 3;
      CF_DEBUG("XXX image.channels()=%d", image.channels());
      if( image.channels() < 4 ) {
        CF_ERROR("Channel 3 not exists in input image");
        return false;
      }
      CF_DEBUG("YYY image.channels()=%d", image.channels());
      break;

    case COLOR_CHANNEL_YCrCb:
      if( image.channels() != 3 ) {
        CF_ERROR("3-channel BGR input image expected");
        return false;
      }

      cc = 0;
      cv::cvtColor(image, image, cv::COLOR_BGR2YCrCb);
      break;

    case COLOR_CHANNEL_Lab:
      if( image.channels() != 3 ) {
        CF_ERROR("3-channel BGR input image expected");
        return false;
      }
      cc = 0;
      cscale = 100.0;
      cv::cvtColor(image, image, cv::COLOR_BGR2Lab);
      break;

    case COLOR_CHANNEL_Luv:
      if( image.channels() != 3 ) {
        CF_ERROR("3-channel BGR input image expected");
        return false;
      }
      cc = 0;
      cscale = 100.0;
      cv::cvtColor(image, image, cv::COLOR_BGR2Luv);
      break;

    case COLOR_CHANNEL_HSV:
      if( image.channels() != 3 ) {
        CF_ERROR("3-channel BGR input image expected");
        return false;
      }
      cc = 2;
      cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
      break;

    case COLOR_CHANNEL_HLS:
      if( image.channels() != 3 ) {
        CF_ERROR("3-channel BGR input image expected");
        return false;
      }
      cc = 1;
      cv::cvtColor(image, image, cv::COLOR_BGR2HLS);
      break;

    default:
      CF_ERROR("APP BUG: Invalid color channel requested: %d", _channel);
      return false;
  }

  cv::extractChannel(image, tmp, cc);

  if( _sigma > 0 && _alpha > 0 && _alpha < 1 ) {
    unsharp_mask(tmp, _ignore_mask ? cv::noArray() : mask, tmp, _sigma, _alpha);
  }

  if( _blur_color_channels > 0 ) {
    cv::GaussianBlur(image, image, cv::Size(-1, -1),
        _blur_color_channels, _blur_color_channels,
        cv::BORDER_REPLICATE);
  }

  cv::insertChannel(tmp, image, cc);

  switch (_channel) {

    case COLOR_CHANNEL_YCrCb:
      cv::cvtColor(image, image, cv::COLOR_YCrCb2BGR);
      break;

    case COLOR_CHANNEL_Lab:
      cv::cvtColor(image, image, cv::COLOR_Lab2BGR);
      break;

    case COLOR_CHANNEL_Luv:
      cv::cvtColor(image, image, cv::COLOR_Luv2BGR);
      break;

    case COLOR_CHANNEL_HSV:
      cv::cvtColor(image, image, cv::COLOR_HSV2BGR);
      break;

    case COLOR_CHANNEL_HLS:
      cv::cvtColor(image, image, cv::COLOR_HLS2BGR);
      break;
  }

  if ( _outmax > _outmin ) {
    cv::max(image, _outmin, image);
    cv::min(image, _outmax, image);
  }

  return true;
}
