/*
 * c_lpg_sharpness_measure.h
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_lpg_sharpness_measure_h__
#define __c_lpg_sharpness_measure_h__

#include "c_image_sharpness_measure.h"

class c_lpg_sharpness_measure:
    public c_image_sharpness_measure
{
public:
  typedef c_lpg_sharpness_measure this_class;
  typedef c_image_sharpness_measure base;

  void set_laplacian_weight(double v);
  double laplacian_weight() const;

  void set_gradient_weight(double v);
  double gradient_weight() const;

  void set_dscale(int v);
  int dscale() const;

  void set_avgchannel(bool v);
  bool avgchannel() const;

  cv::Scalar compute(cv::InputArray image) override;
  cv::Scalar create_sharpeness_map(cv::InputArray image, cv::OutputArray output_map) override;

  static cv::Scalar compute_lpg_map(cv::InputArray image, cv::OutputArray output_map,
      double lw, double gw, int dscale, bool avgchannel);

protected:
  double laplacian_weight_ = 1;
  double gradient_weight_ = 1;
  int dscale_ = 1;
  bool avgchannel_ = true;

};

#endif /* __c_lpg_sharpness_measure_h__ */
