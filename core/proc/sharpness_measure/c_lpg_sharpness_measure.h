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


struct c_lpg_options
{
  double k = 1;
  double p = 4;
  int dscale = 1;
  int uscale = 3;
  bool avgchannel = true;
};

class c_lpg_sharpness_measure:
    public c_image_sharpness_measure
{
public:
  typedef c_lpg_sharpness_measure this_class;
  typedef c_image_sharpness_measure base;

  c_lpg_sharpness_measure();
  c_lpg_sharpness_measure(const c_lpg_options & opts);

  void set_k(double v);
  double k() const;

  void set_p(double v);
  double p() const;

  void set_dscale(int v);
  int dscale() const;

  void set_uscale(int v);
  int uscale() const;

  void set_avgchannel(bool v);
  bool avgchannel() const;

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const override;
  static bool compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map,
      double k, double p, int dscale, int uscale, bool avgchannel,
      cv::Scalar * output_sharpness_metric);
  static bool compute(cv::InputArray image, cv::InputArray mask , cv::OutputArray output_map,
      const c_lpg_options & opts,
      cv::Scalar * output_sharpness_metric);

  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;
  static bool create_map(cv::InputArray image, cv::OutputArray output_map,
      const c_lpg_options & opts);

protected:
  c_lpg_options options_;
};

#endif /* __c_lpg_sharpness_measure_h__ */
