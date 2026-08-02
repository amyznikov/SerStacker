/*
 * c_local_variance_sharpness_measure.h
 *
 *  Created on: Jul 30, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_local_variance_sharpness_measure_h__
#define __c_local_variance_sharpness_measure_h__

#include "c_image_sharpness_measure.h"
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

struct c_local_variance_map_options
{
  double p = 4;
  int kradius = 5;
  int dscale = 2;
  int uscale = 7;
};

bool serialize_local_variance_map_options(c_config_setting section, bool save, c_local_variance_map_options & opts);

inline bool save_settings(c_config_setting section, const c_local_variance_map_options & opts)
{
  return serialize_local_variance_map_options(section, true,
      const_cast<c_local_variance_map_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_local_variance_map_options * opts)
{
  return serialize_local_variance_map_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_local_variance_map_options> & ctx)
{
  using S = c_local_variance_map_options;
  ctlbind(ctls, "p", ctx(&S::p), "Sharpness map power");
  ctlbind(ctls, "kradius", ctx(&S::kradius), "Morphological gradient SE radius");
  ctlbind(ctls, "dscale", ctx(&S::dscale), "Downscale pyramid level");
  ctlbind(ctls, "uscale", ctx(&S::uscale), "Upscale pyramid level");
}

class c_local_variance_sharpness_measure :
    public c_image_sharpness_measure
{
public:
  c_local_variance_sharpness_measure()
  {
  }

  explicit c_local_variance_sharpness_measure(const c_local_variance_map_options & opts) :
      _opts(opts)
  {
  }

  void set_opts(const c_local_variance_map_options & opts)
  {
    _opts = opts;
  }

  const c_local_variance_map_options & opts() const
  {
    return _opts;
  }

  c_local_variance_map_options & opts()
  {
    return _opts;
  }

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const final;
  bool create_map(cv::InputArray image, cv::OutputArray output_map) const final;
  static bool create_map(cv::InputArray image, cv::OutputArray output_map,
      const c_local_variance_map_options & opts);

protected:
  c_local_variance_map_options _opts;
};

bool create_local_variance_map(cv::InputArray _src, cv::OutputArray dst,
    const c_local_variance_map_options & opts);

bool create_morph_gradient_map(cv::InputArray src, cv::OutputArray dst,
    const c_local_variance_map_options & opts);

#endif /* __c_local_variance_sharpness_measure_h__ */
