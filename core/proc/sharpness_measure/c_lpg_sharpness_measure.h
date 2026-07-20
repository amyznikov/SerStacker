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
#include <core/ctrlbind/ctrlbind.h>
#include <core/proc/lpg.h>
#include <core/settings.h>

struct c_lpg_options
{
  double k = 2;
  double p = 2;
  int dscale = 2;
  int uscale = 6;
};

bool serialize_lpg_options(c_config_setting section, bool save, c_lpg_options & opts);
bool load_settings(c_config_setting settings, c_lpg_options * cfg);
bool save_settings(c_config_setting settings, const c_lpg_options & c);

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_lpg_options> & ctx)
{
  using S = c_lpg_options;
  ctlbind(ctls, "k", ctx(&S::k), "Sharpness map: (k * laplacian + gradient) / (k + 1)");
  ctlbind(ctls, "p", ctx(&S::p), "Sharpness estimator: sharpness map power");
  ctlbind(ctls, "dscale", ctx(&S::dscale), "Sharpness estimator: sharpness map downscale pyramid level");
  ctlbind(ctls, "uscale", ctx(&S::uscale), "Sharpness estimator: sharpness map upscale pyramid level");
}

class c_lpg_sharpness_measure:
    public c_image_sharpness_measure
{
public:
  typedef c_lpg_sharpness_measure this_class;
  typedef c_image_sharpness_measure base;

  c_lpg_sharpness_measure();
  c_lpg_sharpness_measure(const c_lpg_options & opts);

  c_lpg_options & options();
  const c_lpg_options & options() const;

  void set_k(double v);
  double k() const;

  void set_p(double v);
  double p() const;

  void set_dscale(int v);
  int dscale() const;

  void set_uscale(int v);
  int uscale() const;

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const override;
  static bool compute(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map,
      double k, double p, int dscale, int uscale,
      cv::Scalar * output_sharpness_metric);
  static bool compute(cv::InputArray image, cv::InputArray mask , cv::OutputArray output_map,
      const c_lpg_options & opts,
      cv::Scalar * output_sharpness_metric);

  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;
  static bool create_map(cv::InputArray image, cv::OutputArray output_map,
      const c_lpg_options & opts);

//  std::string save_settings();
//  bool load_settings(const std::string & text);
//  bool serialize(c_config_setting settings, bool save);

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_lpg_options _opts;
};

inline bool lpg(cv::InputArray image, cv::InputArray mask, cv::OutputArray output_map, const c_lpg_options & opts )
{
  return lpg(image, mask, output_map, opts.k, opts.p, opts.dscale, opts.uscale);
}

#endif /* __c_lpg_sharpness_measure_h__ */
