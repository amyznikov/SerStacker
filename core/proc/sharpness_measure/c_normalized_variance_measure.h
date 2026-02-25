/*
 * c_normalized_variance_measure.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 *
 * Normalized image variance from paper
 *  "Autofocusing Algorithm Selection in Computer Microscopy"
 *    by Yu Sun and Stefan Duthaler and Bradley J. Nelson
 *
 *  measure = stdev(image) / mean(image)
 */

#pragma once
#ifndef __c_normalized_variance_measure_h__
#define __c_normalized_variance_measure_h__

#include "c_image_sharpness_measure.h"
#include <core/ctrlbind/ctrlbind.h>

struct c_normalized_variance_measure_options
{
  bool avgchannel = true;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_normalized_variance_measure_options> & ctx)
{
  using S = c_normalized_variance_measure_options;
  ctlbind(ctls, "avgchannel", ctx(&S::avgchannel), "");
}


class c_normalized_variance_measure :
    public c_image_sharpness_measure
{
public:
  typedef c_normalized_variance_measure this_class;
  typedef c_image_sharpness_measure base;

  void set_avgchannel(bool v);
  bool avgchannel() const;

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const override;
  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static bool compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map, bool avgchannel,
      cv::Scalar * output_sharpness_metric);

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_normalized_variance_measure_options _opts;
};

#endif /* __c_normalized_variance_measure_h__ */
