/*
 * c_crop_image_routine.cc
 *
 *  Created on: Aug 31, 2023
 *      Author: amyznikov
 */

#include "c_crop_image_routine.h"

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

  cv::Mat1b m(size, 255);
  cv::rectangle(m, cv::Rect(4, 2, size.width-8, size.height-4), 0, -1);

  image.setTo(0, m);

  return true;
}
