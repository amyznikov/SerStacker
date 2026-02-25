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
#include <core/ctrlbind/ctrlbind.h>

struct c_harris_sharpness_measure_options
{
  double k = 5;
  int dscale = 1;
  int uscale = 1;
  bool avgchannel = true;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_harris_sharpness_measure_options> & ctx)
{
  using S = c_harris_sharpness_measure_options;
  ctlbind(ctls, "k", ctx(&S::k), "Metric: Determinant + k * Trace^2");
  ctlbind(ctls, "dscale", ctx(&S:: dscale), "");
  ctlbind(ctls, "uscale", ctx(&S::uscale), "");
  ctlbind(ctls, "avgchannel", ctx(&S:: avgchannel), "");
}


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

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const override;
  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static bool compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map,
      double k, int dscale, int uscale, bool avgchannel,
      cv::Scalar * output_sharpness_measure = nullptr);

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_harris_sharpness_measure_options _opts;
};

#endif /* __c_harris_sharpness_measure_h__ */
