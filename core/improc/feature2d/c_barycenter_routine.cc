/*
 * c_barycenter_routine.cc
 *
 *  Created on: May 30, 2026
 *      Author: amyznikov
 */

#include "c_barycenter_routine.h"
#include <core/proc/reduce_channels.h>

static bool adjust_roi(const cv::Rect & src, const cv::Size & image_size, cv::Rect * dst)
{
  const int l = (std::min)(image_size.width - 1, (std::max)(0, src.x));
  const int t = (std::min)(image_size.height - 1, (std::max)(0, src.y));
  const int r = (std::min)(image_size.width - 1, (std::max)(0, src.x + src.width - 1));
  const int b = (std::min)(image_size.height - 1, (std::max)(0, src.y + src.height - 1));
  *dst = cv::Rect(l, t, r - l + 1, b - t + 1);
  return !dst->empty();
}

static cv::Point2d baryCenter(cv::InputArray image)
{
  const cv::Moments m = cv::moments(image.getMat());
  return cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
}

void c_barycenter_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "update pointer", ctx(&this_class::_update_pointer), "");
  ctlbind(ctls, "ignore mask", ctx(&this_class::_ignore_mask), "");
}

bool c_barycenter_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _update_pointer);
    return true;
  }
  return false;
}

bool c_barycenter_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Rect rc;
  cv::Point2d bc;

  if ( !ctlbind_get_roi(&rc) || !adjust_roi(rc, image.size(), &rc) ) {
    CF_ERROR("Empty or invalid roi specified");
    return false;
  }

  if( _ignore_mask || mask.empty() ) {
    const cv::Mat img = image.getMat();
    bc = baryCenter(img(rc));
  }
  else {

    cv::Mat img, imask;

    if ( mask.channels() == 1 ) {
      cv::bitwise_not(mask, imask);
    }
    else {
      reduce_color_channels(mask, imask, cv::REDUCE_MIN);
      cv::bitwise_not(imask, imask);
    }

    if ( image.channels() == 1 ) {
      image.getMat().copyTo(img, mask);
    }
    else {
      cv::cvtColor(image, img, cv::COLOR_BGR2GRAY);
      img.setTo(0, imask);
    }
    bc = baryCenter(img(rc));
  }

  CF_DEBUG("bc: roi=(%g; %g) image=(%g;%g)", bc.x, bc.y, bc.x + rc.x, bc.y + rc.y);
  if ( _update_pointer ) {
    ctlbind_update_arrow(bc.x + rc.x, bc.y + rc.y);
  }

  return true;
}

