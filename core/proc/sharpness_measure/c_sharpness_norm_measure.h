/*
 * c_sharpness_norm_measure.h
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 *
 *  Computes the norm of difference between image and its low-pass filtered version
 */

#pragma once
#ifndef __c_sharpness_norm_measure_h__
#define __c_sharpness_norm_measure_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>

struct c_sharpness_norm_measure_options
{
  double sigma = 1; // M_SQRT2;
  cv::NormTypes norm_type = cv::NORM_L1;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_sharpness_norm_measure_options> & ctx)
{
  using S = c_sharpness_norm_measure_options;
  ctlbind(ctls, "sigma", ctx(&S::sigma), "");
  ctlbind(ctls, "norm_type", ctx(&S::norm_type), "");
}


class c_sharpness_norm_measure
{
public:
  typedef c_sharpness_norm_measure this_class;
  typedef std::shared_ptr<this_class> ptr;

  void set_norm_type(cv::NormTypes v);
  cv::NormTypes norm_type() const;

  void set_sigma(double v);
  double sigma() const;

  double measure(cv::InputArray src, cv::InputArray mask = cv::noArray()) const;
  static double measure(cv::InputArray src, cv::InputArray mask, double sigma, cv::NormTypes norm_type);

  double add(cv::InputArray src, cv::InputArray mask = cv::noArray());
  double average() const;
  void reset();

  double accumulator() const;
  int counter() const;

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_sharpness_norm_measure_options _opts;
  double _accumulator = 0;
  int _counter = 0;
};

#endif /* __c_sharpness_norm_measure_h__ */
