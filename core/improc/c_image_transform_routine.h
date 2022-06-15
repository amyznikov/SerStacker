/*
 * c_image_transform_routine.h
 *
 *  Created on: Jun 12, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_transform_routine_h__
#define __c_image_transform_routine_h__

#include "c_image_processor.h"

class c_image_transform_routine:
    public c_image_processor_routine
{
public:
  typedef c_image_transform_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  enum image_resize_mode {
    resize_keep,
    resize_expand,
  };

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("transform", "geometrical image transform", "geometrical image transform",
            factory([]() {return ptr(new this_class());}))
    {
    }
  } class_factory;


  c_image_transform_routine(bool enabled = true);

  static ptr create(bool enabled = true);


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

  bool serialize(c_config_setting settings) const override;
  bool deserialize(c_config_setting settings) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, translation, "image translation in pixels before rotation/scale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, rotation, "rotation angle in degrees");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scale, "image scale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, resize_mode, "resize mode");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, interpolation, "interpolation");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, border_type, "border_type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, border_value, "border_value");
  }


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
};

template<> const c_enum_member *
members_of<c_image_transform_routine::image_resize_mode>();

#endif /* __c_image_transform_routine_h__ */
