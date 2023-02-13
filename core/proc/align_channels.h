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

// #include "eccalign.h"

#include <core/proc/image_registration/image_transform.h>
#include <core/proc/image_registration/ecc2.h>

class c_align_color_channels {

public:
  c_align_color_channels() = default;

  void set_motion_type(IMAGE_MOTION_TYPE motion_type);
  IMAGE_MOTION_TYPE motion_type() const;

  void set_interpolation(enum ecc2::ECC_INTERPOLATION_METHOD method);
  enum ecc2::ECC_INTERPOLATION_METHOD interpolation() const;

  void set_border_mode(enum ecc2::ECC_BORDER_MODE border_mode);
  enum ecc2::ECC_BORDER_MODE border_mode() const;

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

  double computed_rho(int channel_index) const;
  double computed_eps(int channel_index) const;
  int computed_iterations(int channel_index) const;
  const c_image_transform::sptr & computed_transform(int channel_index) const;

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
  IMAGE_MOTION_TYPE motion_type_ = IMAGE_MOTION_TRANSLATION;
  enum ecc2::ECC_INTERPOLATION_METHOD interpolation_ = ecc2::ECC_INTER_LINEAR;
  enum ecc2::ECC_BORDER_MODE border_mode_ = ecc2::ECC_BORDER_REPLICATE;
  cv::Scalar border_value_ = cv::Scalar();
  int max_iterations_ = 30;
  double smooth_sigma_ = 1;
  double eps_ = 0.1;
  double update_step_scale_ = 1;

  std::vector<double> computed_eps_;
  std::vector<double> computed_rhos_;
  std::vector<int> computed_iterations_;
  std::vector<c_image_transform::sptr> computed_transforms_;
};

#endif /* __align_channels_h__ */
