/*
 * focus.h
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __focus_h__
#define __focus_h__

#include <opencv2/opencv.hpp>

class c_image_sharpness_measure
{
public:
  typedef c_image_sharpness_measure this_class;

  virtual ~c_image_sharpness_measure() = default;

  virtual cv::Scalar compute(cv::InputArray image) = 0;
};

class c_local_contrast_measure:
    public c_image_sharpness_measure
{
public:
  typedef c_local_contrast_measure this_class;
  typedef c_image_sharpness_measure base;

  cv::Scalar compute(cv::InputArray image) override;

  static cv::Scalar compute_contrast_map(cv::InputArray image, cv::OutputArray cmap,
      double noise_eps = 1e-6,
      cv::InputArray H = cv::noArray());

protected:
  double noise_eps_ = 1e-6;
};

#endif /* __focus_h__ */

