/*
 * c_affine_transform_routine.h
 *
 *  Created on: Jun 12, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_transform_routine_h__
#define __c_image_transform_routine_h__

#include <core/improc/c_image_processor.h>

class c_affine_transform_routine:
    public c_image_processor_routine
{
public:
  enum image_resize_mode {
    resize_keep,
    resize_adjust,
    resize_scale,
  };

  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_affine_transform_routine,
      "c_affine_transform", "Apply affine transform to image");

  void set_rotation(double v);
  double rotation() const;

  void set_translation(const cv::Point2f & v);
  const cv::Point2f & translation() const;

  void set_scale(const cv::Size2f & v);
  const cv::Size2f & scale() const;

  void set_resize_mode(image_resize_mode v);
  image_resize_mode resize_mode() const;

  void set_interpolation(cv::InterpolationFlags v);
  cv::InterpolationFlags interpolation() const;

  void set_border_type(cv::BorderTypes v);
  cv::BorderTypes border_type() const;

  void set_border_value(const cv::Scalar & v);
  const cv::Scalar & border_value() const;

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

  static bool create_transformation_remap(cv::Mat2f & dst,
      const cv::Size & src_image_size,
      double rotation_degrees,
      const cv::Point2f & translation,
      const cv::Size2f & scale,
      image_resize_mode resize_mode);


  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);


protected:
  // new_image = scale * (rotation * (image - center) + center) + translation
  double _rotation = 0;
  cv::Point2f _translation = cv::Point2f(0, 0);
  cv::Size2f _scale = cv::Size2f(1, 1);
  image_resize_mode _resize_mode = resize_keep;
  cv::InterpolationFlags _interpolation = cv::INTER_LINEAR;
  cv::BorderTypes _border_type = cv::BORDER_CONSTANT;
  cv::Scalar _border_value;
  cv::Mat2f _remap;
  cv::Size _previous_image_size;
};

template<> const c_enum_member *
members_of<c_affine_transform_routine::image_resize_mode>();

#endif /* __c_image_transform_routine_h__ */
