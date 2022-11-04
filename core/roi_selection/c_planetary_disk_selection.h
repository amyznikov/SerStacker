/*
 * c_planetary_disk_selection.h
 *
 *  Created on: Jul 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_planetary_disk_selection__
#define __c_planetary_disk_selection__

#include "c_roi_selection.h"

class c_planetary_disk_selection
    : public c_roi_selection
{
public:
  typedef c_planetary_disk_selection this_class;
  typedef c_roi_selection base;
  typedef std::shared_ptr<this_class> ptr;

  c_planetary_disk_selection();
  c_planetary_disk_selection(const cv::Size & crop_size, double gbsigma, double stdev_factor);

  static this_class::ptr create();
  static this_class::ptr create(const cv::Size & crop_size, double gbsigma, double stdev_factor);

  const cv::Size & crop_size() const;
  void set_crop_size(const cv::Size & size) ;

  void set_gbsigma(double v);
  double gbsigma() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  bool select(cv::InputArray image, cv::InputArray image_mask,
      cv::Rect & outputROIRectangle ) override;

  const cv::Point2f & detected_object_position() const;
  const cv::Rect & detected_object_roi()  const;

//  bool detect_object_roi(cv::InputArray image, cv::InputArray image_mask,
//      cv::Point2f & outputObjectLocation,
//      cv::Rect & outputCropRect ) override;

protected:
  cv::Size crop_size_;
  double gbsigma_ = 1;
  double stdev_factor_ = 0.5;
  cv::Point2f objpos_;
  cv::Rect objrect_;
};

#endif /* __c_planetary_disk_selection__ */
