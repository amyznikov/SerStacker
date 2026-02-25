/*
 * c_unkanala_remap_routine.h
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_unkanala_remap_routine_h__
#define __c_unkanala_remap_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/unkanala.h>


class c_unkanala_remap_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_unkanala_remap_routine,
      "unkanala_remap", "Apply unkanala_remap to image");

  const cv::Size & image_size() const
  {
    return _intrinsics.image_size;
  }

  void set_image_size(const cv::Size & v)
  {
    _intrinsics.image_size = v;
    _remap.release();
  }

  double focal_length_x() const
  {
    return _intrinsics.focal_length_x;
  }

  void set_focal_length_x(double v)
  {
    _intrinsics.focal_length_x = v;
    _remap.release();
  }

  double focal_length_y() const
  {
    return _intrinsics.focal_length_y;
  }

  void set_focal_length_y(double v)
  {
    _intrinsics.focal_length_y = v;
    _remap.release();
  }


  double principal_point_x() const
  {
    return _intrinsics.principal_point_x;
  }

  void set_principal_point_x(double v)
  {
    _intrinsics.principal_point_x = v;
    _remap.release();
  }

  double principal_point_y () const
  {
    return _intrinsics.principal_point_y;
  }

  void set_principal_point_y (double v)
  {
    _intrinsics.principal_point_y = v;
    _remap.release();
  }

  const std::vector<double> & distortion_coefficients() const
  {
    return _intrinsics.distortion_coefficients;
  }

  void set_distortion_coefficients(const std::vector<double> & v)
  {
    _intrinsics.distortion_coefficients = v;
    _remap.release();
  }

  bool serialize(c_config_setting settings, bool save);
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_kanala_intrinsics _intrinsics;
  cv::Mat2f _remap;
};

#endif /* __c_unkanala_remap_routine_h__ */
