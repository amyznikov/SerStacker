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

  void set_equalize_hist(bool v);
  bool equalize_hist() const;

  static cv::Scalar compute_contrast_map(cv::InputArray image, cv::OutputArray cmap,
      double eps = 1e-6, int dscale = 0, bool equalize_hist = false);

protected:
  int dscale_ = 0;
  double eps_ = 1e-6;
  bool equalize_hist_ = false;
};

#endif /* __focus_h__ */

