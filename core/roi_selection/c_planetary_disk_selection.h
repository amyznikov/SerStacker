/*
 * c_planetary_disk_selection.h
 *
 *  Created on: Jul 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_planetary_disk_selection__
#define __c_planetary_disk_selection__

#include "c_feature_based_roi_selection.h"

class c_planetary_disk_selection :
    public c_feature_based_roi_selection
{
public:
  typedef c_planetary_disk_selection this_class;
  typedef c_feature_based_roi_selection base;
  typedef std::shared_ptr<this_class> ptr;

  c_planetary_disk_selection() = default;

  static this_class::ptr create();
  static this_class::ptr create(const cv::Size & crop_size);

  bool detect_object_roi(cv::InputArray image, cv::InputArray image_mask,
      cv::Point2f & outputObjectLocation,
      cv::Rect & outputCropRect ) override;

};

#endif /* __c_planetary_disk_selection__ */
