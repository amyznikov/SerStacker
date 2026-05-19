/*
 * c_roi_rectangle_selection.h
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef CORE_ROI_SELECTION_C_ROI_RECTANGLE_SELECTION_H_
#define CORE_ROI_SELECTION_C_ROI_RECTANGLE_SELECTION_H_

#include "c_roi_selection.h"

class c_roi_rectangle_selection :
    public c_roi_selection
{
public:
  typedef c_roi_rectangle_selection this_class;
  typedef c_roi_selection base;
  typedef std::shared_ptr<this_class> ptr;

  c_roi_rectangle_selection();
  c_roi_rectangle_selection(const cv::Rect & rc);
  c_roi_rectangle_selection(const cv::Point & org, const cv::Size & size);

  static this_class::ptr create();
  static this_class::ptr create(const cv::Rect & rc);
  static this_class::ptr create(const cv::Point & org, const cv::Size & size);

  const cv::Rect & rect() const;
  void set_rect(const cv::Rect & rc);

  cv::Point origin() const;
  void set_origin(const cv::Point & org);

  cv::Size size() const;
  void set_size(const cv::Size & size);

  bool select(cv::InputArray image, cv::InputArray image_mask,
      cv::Rect & outputROIRectangle ) override;

protected:
  cv::Rect rect_;
};

#endif /* CORE_ROI_SELECTION_C_ROI_RECTANGLE_SELECTION_H_ */
