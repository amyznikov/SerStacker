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
#include <core/proc/extract_channel.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

struct c_local_variance_map_options
{
  enum color_channel_type channel = color_channel_gray;
  int kradius = 2;
  int dscale = 1;
  int uscale = 3;
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
  ctlbind(ctls, "channel", ctx(&S::channel), "Intensity channel");
  ctlbind(ctls, "dscale", ctx(&S::dscale), "Downscale pyramid level");
  ctlbind(ctls, "kradius", ctx(&S::kradius), "Morphological gradient SE radius");
  ctlbind(ctls, "uscale", ctx(&S::uscale), "Upscale (smoothing) pyramid level");
}

template<class RootObjectType>
inline void ctlbind_local_variance_estimate_options(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_local_variance_map_options> & ctx)
{
  using S = c_local_variance_map_options;
  ctlbind(ctls, "channel", ctx(&S::channel), "Intensity channel");
  ctlbind(ctls, "dscale", ctx(&S::dscale), "Downscale pyramid level");
  ctlbind(ctls, "kradius", ctx(&S::kradius), "Morphological gradient SE radius");
}

class c_local_variance_sharpness_measure :
    public c_image_sharpness_measure
{
public:
  c_local_variance_sharpness_measure()
  {
  }

  explicit c_local_variance_sharpness_measure(const c_local_variance_map_options & _opts) :
      opts(_opts)
  {
  }

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const final;
  bool create_map(cv::InputArray image, cv::OutputArray outputMap) const final;
  static bool create_map(cv::InputArray image, cv::OutputArray outputMap,
      const c_local_variance_map_options & opts);

public: // Made public to simplify control bindings
  c_local_variance_map_options opts;
};

double compute_local_variance_map(cv::InputArray _src, const c_local_variance_map_options & opts,
    cv::OutputArray outputMap = cv::noArray(), bool returnFullResoltionMap = true);

bool upscale_local_variance_map(cv::Mat & map, const cv::Size & dstSize);

#endif /* __c_local_variance_sharpness_measure_h__ */
