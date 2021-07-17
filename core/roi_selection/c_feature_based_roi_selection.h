/*
 * c_feature_based_roi_selection.h
 *
 *  Created on: Jul 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_feature_based_roi_selection_h__
#define __c_feature_based_roi_selection_h__

#include <opencv2/opencv.hpp>

class c_feature_based_roi_selection {
public:
  typedef c_feature_based_roi_selection this_class;
  typedef std::shared_ptr<this_class> ptr;

  c_feature_based_roi_selection() = default;
  virtual ~c_feature_based_roi_selection() = default;

  void set_crop_size(const cv::Size & size) {
    crop_size_ = size;
  }

  const cv::Size & crop_size() const {
    return crop_size_;
  }

  virtual bool detect_object_roi(cv::InputArray image, cv::InputArray image_mask,
      cv::Point2f & outputObjectLocation,
      cv::Rect & outputCropRect ) = 0;

protected:
  cv::Size crop_size_;
};

#endif /* __c_feature_based_roi_selection_h__ */
