/*
 * c_local_contrast_measure.h
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_local_contrast_measure_h__
#define __c_local_contrast_measure_h__

#include "c_image_sharpness_measure.h"
#include <core/ctrlbind/ctrlbind.h>


struct c_local_contrast_measure_options
{
  int dscale = 1;
  double eps = 1e-3;
  bool avgchannel = true;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_local_contrast_measure_options> & ctx)
{
  using S = c_local_contrast_measure_options;

  ctlbind(ctls, "dscale", ctx(&S::dscale), "");
  ctlbind(ctls, "eps", ctx(&S::eps), "");
  ctlbind(ctls, "avgchannel", ctx(&S::avgchannel), "");
}

class c_local_contrast_measure:
    public c_image_sharpness_measure
{
public:
  typedef c_local_contrast_measure this_class;
  typedef c_image_sharpness_measure base;

  void set_dscale(int v);
  int dscale() const;

  void set_eps(double v);
  double eps() const;

  void set_avgchannel(bool v);
  bool avgchannel() const;

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const override;
  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static bool compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_contrast_map,
      double eps = 1e-3, int dscale = 1, bool avgchannel = false,
      cv::Scalar * output_sharpness_measure = nullptr);

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_local_contrast_measure_options _opts;
};

#endif /* __c_local_contrast_measure_h__ */
