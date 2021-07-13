/*
 * align_channels.h
 *
 *  Created on: Jul 11, 2021
 *      Author: amyznikov
 *
 *   Use of eccalign for color channels alignment
 */

#pragma once
#ifndef __align_channels_h__
#define __align_channels_h__

#include "eccalign.h"

class c_align_color_channels {

public:
  c_align_color_channels() = default;

  void set_motion_type(ECC_MOTION_TYPE motion_type);
  ECC_MOTION_TYPE motion_type() const;

  void set_interpolation_flags(cv::InterpolationFlags flags);
  cv::InterpolationFlags interpolation_flags() const;

  void set_border_value(const cv::Scalar & border_value);
  const cv::Scalar & border_value() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_smooth_sigma(double v);
  double smooth_sigma() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  void set_eps(double v);
  double eps(double v);

  double computed_rho(int channel_index) const;
  double computed_eps(int channel_index) const;
  int computed_iterations(int channel_index) const;
  const cv::Mat & computed_transform(int channel_index) const;

  c_ecc_forward_additive & ecc();
  const c_ecc_forward_additive & ecc() const;

  bool align(int reference_channel_index,
      cv::InputArray src, cv::OutputArray dst,
      cv::InputArray srcmask = cv::noArray(),
      cv::OutputArray dstmask = cv::noArray() );

  bool align(cv::InputArray single_channel_reference_image,
      cv::InputArray src, cv::OutputArray dst,
      cv::InputArray reference_mask = cv::noArray(),
      cv::InputArray srcmask = cv::noArray(),
      cv::OutputArray dstmask = cv::noArray() );

protected:
  c_ecc_forward_additive ecc_;
  std::vector<double> computed_eps_;
  std::vector<double> computed_rhos_;
  std::vector<int> computed_iterations_;
  std::vector<cv::Mat> computed_transforms_;
  cv::Scalar border_value_ = cv::Scalar();
};

#endif /* __align_channels_h__ */
