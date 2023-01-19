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

  void set_dscale(int v);
  int dscale() const;

  void set_eps(double v);
  double eps() const;

  void set_avgchannel(bool v);
  bool avgchannel() const;

  static cv::Scalar compute_contrast_map(cv::InputArray image, cv::OutputArray output_contrast_map,
      double eps = 1e-3, int dscale = 1, bool avgchannel = false);


protected:
  int dscale_ = 1;
  double eps_ = 1e-3;
  bool avgchannel_ = true;
};

#endif /* __focus_h__ */

