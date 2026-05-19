/*
 * c_roi_rectangle_selection.cc
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#include "c_roi_rectangle_selection.h"

c_roi_rectangle_selection::c_roi_rectangle_selection()
{
}

c_roi_rectangle_selection::c_roi_rectangle_selection(const cv::Rect & rc)
  : rect_(rc)
{
}

c_roi_rectangle_selection::c_roi_rectangle_selection(const cv::Point & org, const cv::Size & size)
  : rect_(org, size)
{
}

c_roi_rectangle_selection::ptr c_roi_rectangle_selection::create()
{
  return ptr(new this_class());
}

c_roi_rectangle_selection::ptr c_roi_rectangle_selection::create(const cv::Rect & rc)
{
  return ptr(new this_class(rc));
}

c_roi_rectangle_selection::ptr c_roi_rectangle_selection::create(const cv::Point & org, const cv::Size & size)
{
  return ptr(new this_class(org, size));
}

const cv::Rect & c_roi_rectangle_selection::rect() const
{
  return rect_;
}

void c_roi_rectangle_selection::set_rect(const cv::Rect & rc)
{
  rect_ = rc;
}

cv::Point c_roi_rectangle_selection::origin() const
{
  return cv::Point(rect_.x, rect_.y);
}

void c_roi_rectangle_selection::set_origin(const cv::Point & org)
{
  rect_.x = org.x;
  rect_.y = org.y;
}

cv::Size c_roi_rectangle_selection::size() const
{
  return rect_.size();
}

void c_roi_rectangle_selection::set_size(const cv::Size & size)
{
  rect_.width = size.width;
  rect_.height = size.height;
}

bool c_roi_rectangle_selection::select(cv::InputArray image, cv::InputArray image_mask, cv::Rect & outrc )
{
  outrc = rect_;

  if ( rect_.x < 0 || rect_.x + rect_.width >= image.cols() ) {
    return false;
  }
  if ( rect_.y < 0 || rect_.y + rect_.height >= image.rows() ) {
    return false;
  }

  return true;
}

