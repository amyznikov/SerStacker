/*
 * c_laplacian_sharpness_measure.h
 *
 *  Created on: Sep 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_laplacian_sharpness_measure_h__
#define __c_laplacian_sharpness_measure_h__

#include "c_image_sharpness_measure.h"
#include <core/ctrlbind/ctrlbind.h>

struct c_laplacian_sharpness_measure_options
{
  cv::Size se_size = cv::Size(5, 5);
  int dscale = 2;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_laplacian_sharpness_measure_options> & ctx)
{
  using S = c_laplacian_sharpness_measure_options;
  ctlbind(ctls, "dscale", ctx(&S::dscale), "");
  ctlbind(ctls, "se_size", ctx(&S::se_size), "");
}

class c_laplacian_sharpness_measure :
    public c_image_sharpness_measure
{
public:
  typedef c_laplacian_sharpness_measure this_class;
  typedef c_image_sharpness_measure base;

  c_laplacian_sharpness_measure();
  c_laplacian_sharpness_measure(int dscale, const cv::Size & size);

  void set_dscale(int v);
  int dscale() const;

  void set_se_size(const cv::Size & v);
  const cv::Size & se_size() const;

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const override;

  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static bool compute(cv::InputArray image, cv::InputArray mask,
      int dscale, const cv::Size & se_size,
      cv::Scalar * output_sharpness_measure);

  static bool create_map(cv::InputArray image,
      int dscale, const cv::Size & se_size,
      cv::OutputArray output_map);

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_laplacian_sharpness_measure_options _opts;
};

#endif /* __c_laplacian_sharpness_measure_h__ */
