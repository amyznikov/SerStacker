/*
 * c_jovian_rotation_registration.h
 *
 *  Created on: Sep 12, 2021
 *      Author: amyznikov
 *
 *
 *  This is c_frame_registration interface to the
 *  c_jovian_derotation algorithm.
 */

#pragma once
#ifndef __c_jovian_rotation_registration_h__
#define __c_jovian_rotation_registration_h__

#include "c_planetary_disk_registration.h"
#include <core/proc/jupiter.h>

struct c_jovian_derotation_options {
  double min_rotation = -30 * CV_PI / 180;
  double max_rotation = +30 * CV_PI / 180;
  int eccflow_support_scale = 3;
  int eccflow_normalization_scale = 0;
  int eccflow_max_pyramid_level = 1;
  bool align_jovian_disk_horizontally = false;
};

class c_jovian_rotation_registration :
    public c_planetary_disk_registration
{
public:

  typedef c_jovian_rotation_registration this_class;
  typedef c_planetary_disk_registration base;

  typedef std::shared_ptr<this_class> ptr;

  static this_class::ptr create();

  // hmmm... it is questionable if these overloads are really usefull though...
  static this_class::ptr create(const c_jovian_derotation_options & opts);
  static this_class::ptr create(const c_frame_registration_base_options & base_opts,
      const c_planetary_disk_registration_options & planetary_disk_opts,
      const c_jovian_derotation_options & opts);

public: // parameters

  const c_jovian_derotation_options & jovian_derotation_options() const ;
  c_jovian_derotation_options & jovian_derotation_options();

  const c_jovian_derotation & jovian_derotation() const;

public: // overrides

  bool setup_referece_frame(cv::InputArray image,
      cv::InputArray mask = cv::noArray()) override;

  bool register_frame(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray dst = cv::noArray(),
      cv::OutputArray dstmask = cv::noArray()) override;

  bool custom_remap(const cv::Mat2f & rmap,
      cv::InputArray src, cv::OutputArray dst,
      cv::InputArray src_mask = cv::noArray(),
      cv::OutputArray dst_mask = cv::noArray(),
      enum ECC_INTERPOLATION_METHOD interpolation_flags = ECC_INTER_UNKNOWN,
      enum ECC_BORDER_MODE border_mode = ECC_BORDER_UNKNOWN,
      const cv::Scalar & border_value = cv::Scalar()) const override;

  void set_enable_debug(bool v) override;
  void set_debug_path(const std::string & v) override;

protected:
  c_jovian_rotation_registration();
  c_jovian_rotation_registration(const c_jovian_derotation_options & opts);
  c_jovian_rotation_registration(const c_frame_registration_base_options & base_opts,
      const c_planetary_disk_registration_options & planetary_disk_options);
  c_jovian_rotation_registration(const c_frame_registration_base_options & base_opts,
      const c_planetary_disk_registration_options & planetary_disk_options,
      const c_jovian_derotation_options & jovian_rotation_options);

protected:
  c_jovian_derotation derotation_;
  c_jovian_derotation_options jovian_derotation_options_;
};

#endif /* __c_jovian_rotation_registration_h__ */
