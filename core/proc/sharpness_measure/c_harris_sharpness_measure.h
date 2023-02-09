/*
 * c_harris_sharpness_measure.h
 *
 *  Created on: Jan 25, 2023
 *      Author: amyznikov
 *
 *  Experiments (with some improvisations) with approach from paper:
 *
 *  FocusALL: Focal Stacking of Microscopic Images Using Modified Harris Corner Response Measure.
 *   Madhu S. Sigdel,* Madhav Sigdel,* Semih Dinç,* Imren Dinç,* Marc L. Pusey,† and Ramazan S. Aygün*
 *
 *  https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4888603
 *
 *
 * The weighted sum of squares of laplacian and gradient (as in c_lpg_sharpness_measure)
 * still looks much better for me.
 *
 */

#pragma once
#ifndef __c_harris_sharpness_measure_h__
#define __c_harris_sharpness_measure_h__

#include "c_image_sharpness_measure.h"

class c_harris_sharpness_measure:
    public c_image_sharpness_measure
{
public:
  typedef c_harris_sharpness_measure this_class;
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
  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static bool compute(cv::InputArray image, cv::OutputArray output_map,
      double k, int dscale, int uscale, bool avgchannel,
      cv::Scalar * output_sharpness_measure = nullptr);

protected:
  double k_ = 5;
  int dscale_ = 1;
  int uscale_ = 1;
  bool avgchannel_ = true;
};

#endif /* __c_harris_sharpness_measure_h__ */
