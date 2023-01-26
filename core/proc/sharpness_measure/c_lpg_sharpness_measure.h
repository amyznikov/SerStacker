/*
 * c_lpg_sharpness_measure.h
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 *
 * Experiments with weighted sum of squares of laplacian and gradient.
 *
 * Tests on microphotography focal stacking images seems to give good result.
 *
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

  void set_k(double v);
  double k() const;

  void set_dscale(int v);
  int dscale() const;

  void set_uscale(int v);
  int uscale() const;

  void set_avgchannel(bool v);
  bool avgchannel() const;

  cv::Scalar compute(cv::InputArray image) const override;
  cv::Scalar create_sharpeness_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static cv::Scalar create_map(cv::InputArray image, cv::OutputArray output_map,
      double k, int dscale, int uscale, bool avgchannel);

protected:
  double k_ = 6;
  int dscale_ = 1;
  int uscale_ = 2;
  bool avgchannel_ = true;

};

#endif /* __c_lpg_sharpness_measure_h__ */
