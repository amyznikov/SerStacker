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
  ECC_ALIGN_METHOD method = ECC_ALIGN_LM;
  IMAGE_MOTION_TYPE motion_type = IMAGE_MOTION_TRANSLATION;
  enum ECC_INTERPOLATION_METHOD interpolation = ECC_INTER_LINEAR;
  enum ECC_BORDER_MODE border_mode = ECC_BORDER_REPLICATE;
  cv::Scalar border_value = cv::Scalar();
  double smooth_sigma = 0;
  double eps = 0.1;
  double update_step_scale = 1;
  double normalization_eps = 1;
  int max_iterations = 30;
  int max_level = 0;
  int normalization_level = 3;
};

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
  ctlbind(ctls, "normalization_eps", ctx(&S::normalization_eps), "");
  ctlbind(ctls, "normalization_level", ctx(&S::normalization_level), "");
}


/**
 *  Use of ECC for color channels alignment in multi-channel images
 *  */
class c_align_color_channels
{
public:
  typedef c_align_color_channels this_class;

  c_align_color_channels() = default;

  c_align_color_channels_options & options()
  {
    return _opts;
  }

  const c_align_color_channels_options & options() const
  {
    return _opts;
  }

  void set_method(ECC_ALIGN_METHOD v);
  ECC_ALIGN_METHOD method() const;

  void set_motion_type(IMAGE_MOTION_TYPE motion_type);
  IMAGE_MOTION_TYPE motion_type() const;

  void set_interpolation(enum ECC_INTERPOLATION_METHOD method);
  enum ECC_INTERPOLATION_METHOD interpolation() const;

  void set_border_mode(enum ECC_BORDER_MODE border_mode);
  enum ECC_BORDER_MODE border_mode() const;

  void set_border_value(const cv::Scalar & border_value);
  const cv::Scalar & border_value() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_smooth_sigma(double v);
  double smooth_sigma() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  void set_eps(double v);
  double eps() const;

  void set_max_level(int v);
  int max_level() const;

  void set_normalization_level(int v);
  int normalization_level() const;

  void set_normalization_eps(double v);
  double normalization_eps() const;

  const c_image_transform::sptr & computed_transform(int channel_index) const;

  bool align(int reference_channel_index,
      cv::InputArray src,
      cv::OutputArray dst,
      cv::InputArray srcmask = cv::noArray(),
      cv::OutputArray dstmask = cv::noArray() );

  bool align(cv::InputArray single_channel_reference_image,
      cv::InputArray src,
      cv::OutputArray dst,
      cv::InputArray reference_mask = cv::noArray(),
      cv::InputArray srcmask = cv::noArray(),
      cv::OutputArray dstmask = cv::noArray() );

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_align_color_channels_options _opts;
  std::vector<c_image_transform::sptr> _computed_transforms;
};



#endif /* __align_channels_h__ */
