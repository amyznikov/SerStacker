/*
 * c_crop_image_routine.cc
 *
 *  Created on: Aug 31, 2023
 *      Author: amyznikov
 */

#include "c_crop_image_routine.h"

static void adjust_rect(const cv::Rect & src, cv::Rect & dst, const cv::Size & image_size)
{
  const int l = std::max(0, std::min(image_size.width - 1, src.x));
  const int t = std::max(0, std::min(image_size.height - 1, src.y));
  const int r = std::max(0, std::min(image_size.width - 1, src.x + src.width - 1));
  const int b = std::max(0, std::min(image_size.height - 1, src.y + src.height - 1));

  dst.x = l;
  dst.y = t;
  dst.width = r >= 0 ? r - l + 1 : -1;
  dst.height = b >= 0 ? b - t + 1 : -1;
}


void c_crop_image_routine::set_rect(const cv::Rect & rc)
{
  rect_ = rc;
}

const cv::Rect & c_crop_image_routine::rect() const
{
  return rect_;
}

void c_crop_image_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, rect, "crop rectangle in format X;Y;WxH");
}

bool c_crop_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, rect);
    return true;
  }
  return false;
}

bool c_crop_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Size size = image.size();

  CF_DEBUG("rect_: x=%d y=%d w=%d h=%d", rect_.x, rect_.y, rect_.width, rect_.height);

  cv::Rect rc;
  adjust_rect(rect_, rc, size);

  if ( rc.empty() ) {
    CF_ERROR("c_crop_image_routine: adjusted rect is empty");
  }
  else {

    if( !image.empty() ) {
      image.getMat()(rc).copyTo(image);
    }

    if( !mask.empty() ) {
      mask.getMat()(rc).copyTo(mask);
    }
  }

  return true;
}
