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
  const cv::Size2f scale() const;

  void set_resize_mode(image_resize_mode v);
  image_resize_mode resize_mode() const;

  void set_interpolation(cv::InterpolationFlags v);
  cv::InterpolationFlags interpolation() const;

  void set_border_type(cv::BorderTypes v);
  cv::BorderTypes border_type() const;

  void set_border_value(const cv::Scalar & v);
  const cv::Scalar & border_value() const;

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, translation, "image translation in pixels before rotation/scale");
    BIND_PCTRL(ctls, rotation, "rotation angle in degrees");
    BIND_PCTRL(ctls, scale, "image scale");
    BIND_PCTRL(ctls, resize_mode, "resize mode");
    BIND_PCTRL(ctls, interpolation, "interpolation");
    BIND_PCTRL(ctls, border_type, "border_type");
    BIND_PCTRL(ctls, border_value, "border_value");
  }

  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  static bool create_transformation_remap(cv::Mat2f & dst,
      const cv::Size & src_image_size,
      double rotation_degrees,
      const cv::Point2f & translation,
      const cv::Size2f & scale,
      image_resize_mode resize_mode);

protected:


  // new_image = scale * (rotation * (image - center) + center) + translation
  double rotation_ = 0;
  cv::Point2f translation_ = cv::Point2f(0, 0);
  cv::Size2f scale_ = cv::Size2f(1, 1);
  image_resize_mode resize_mode_ = resize_keep;
  cv::InterpolationFlags interpolation_ = cv::INTER_LINEAR;
  cv::BorderTypes border_type_ = cv::BORDER_CONSTANT;
  cv::Scalar border_value_;
  cv::Mat2f remap_;
  cv::Size previous_image_size_;
};

template<> const c_enum_member *
members_of<c_affine_transform_routine::image_resize_mode>();

#endif /* __c_image_transform_routine_h__ */
