/*
 * c_align_color_channels.h
 *
 *  Created on: Jul 11, 2021
 *      Author: amyznikov
 *
 *  Use of ECC for color channels alignment in multi-channel images
 */

#pragma once
#ifndef __align_channels_h__
#define __align_channels_h__

#include <core/ctrlbind/ctrlbind.h>
#include "image_transform.h"
#include "ecc2.h"


struct c_align_color_channels_options
{
  ECC_ALIGN_METHOD method = ECC_ALIGN_INVERSE_COMPOSITIONAL_LM;
  IMAGE_MOTION_TYPE motion_type = IMAGE_MOTION_TRANSLATION;
  enum ECC_INTERPOLATION_METHOD interpolation = ECC_INTER_LINEAR;
  enum ECC_BORDER_MODE border_mode = ECC_BORDER_REPLICATE;
  cv::Scalar border_value = cv::Scalar();
  double smooth_sigma = 0;
  double eps = 0.1;
  double update_step_scale = 1.25;
  int max_iterations = 30;
  int max_level = 0;
  int normalization_level = 3;
};


bool serialize_align_color_channels_options(c_config_setting section, bool save,
    c_align_color_channels_options & opts);

inline bool save_settings(c_config_setting section, const c_align_color_channels_options & opts)
{
  return serialize_align_color_channels_options(section, true,
      const_cast<c_align_color_channels_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_align_color_channels_options * opts)
{
  return serialize_align_color_channels_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_align_color_channels_options> & ctx)
{
  using S = c_align_color_channels_options;
  ctlbind(ctls, "method", ctx(&S::method), "");
  ctlbind(ctls, "motion_type", ctx(&S::motion_type), "");
  ctlbind(ctls, "interpolation", ctx(&S::interpolation), "");
  ctlbind(ctls, "border_mode", ctx(&S::border_mode), "");
  ctlbind(ctls, "border_value", ctx(&S::border_value), "");
  ctlbind(ctls, "smooth_sigma", ctx(&S::smooth_sigma), "");
  ctlbind(ctls, "eps", ctx(&S::eps), "");
  ctlbind(ctls, "update_step_scale", ctx(&S::update_step_scale), "");
  ctlbind(ctls, "max_iterations", ctx(&S::max_iterations), "");
  ctlbind(ctls, "max_level", ctx(&S::max_level), "");
  ctlbind(ctls, "normalization_level", ctx(&S::normalization_level), "");
}


/**
 *  Use of ECCH for color channels alignment in multi-channel images
 *  */
class c_align_color_channels
{
public:
  typedef c_align_color_channels this_class;

  const c_image_transform::sptr & computed_transform(int channel_index) const;

  bool align(cv::InputArray src, cv::OutputArray dst,
      const c_align_color_channels_options & opts,
      int reference_channel_index,
      cv::InputArray srcmask = cv::noArray(),
      cv::OutputArray dstmask = cv::noArray() );

  bool align(cv::InputArray src, cv::OutputArray dst,
      cv::InputArray reference_image, cv::InputArray reference_mask,
      const c_align_color_channels_options & opts,
      cv::InputArray srcmask = cv::noArray(),
      cv::OutputArray dstmask = cv::noArray() );

protected:
  std::vector<c_image_transform::sptr> _image_transforms;
};



#endif /* __align_channels_h__ */
