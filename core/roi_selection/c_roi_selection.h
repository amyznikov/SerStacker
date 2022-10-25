/*
 * c_roi_selection.h
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_roi_selection_h__
#define __c_roi_selection_h__

#include <opencv2/opencv.hpp>

class c_roi_selection {
public:
  typedef c_roi_selection this_class;
  typedef std::shared_ptr<this_class> ptr;

  c_roi_selection() = default;
  virtual ~c_roi_selection() = default;

  virtual bool select(cv::InputArray image, cv::InputArray image_mask,
      cv::Rect & outputROIRectangle ) = 0;
};

#endif /* __c_roi_selection_h__ */
