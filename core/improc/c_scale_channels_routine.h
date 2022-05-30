/*
 * c_scale_channels_routine.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_scale_channels_routine_h__
#define __c_scale_channels_routine_h__

#include "c_image_processor.h"

class c_scale_channels_routine
    : public c_image_processor_routine
{
public:
  typedef c_scale_channels_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("scale_channels", "scale_channels", "scale image channels",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_scale_channels_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  const cv::Scalar & bias() const {
    return bias_;
  }

  void set_bias(const cv::Scalar & v) {
    bias_ = v;
  }

  double bias_r() const {
    return bias_[2];
  }

  void set_bias_r(double v) {
    bias_[2] = v;
  }

  double bias_g() const {
    return bias_[1];
  }

  void set_bias_g(double v) {
    bias_[1] = v;
  }

  double bias_b() const {
    return bias_[0];
  }

  void set_bias_b(double v) {
    bias_[0] = v;
  }

  double bias_a() const {
    return bias_[3];
  }

  void set_bias_a(double v) {
    bias_[3] = v;
  }



  const cv::Scalar & stretch() const {
    return stretch_;
  }

  void set_stretch(const cv::Scalar & v) {
    stretch_ = v;
  }

  double stretch_r() const {
    return stretch_[2];
  }

  void set_stretch_r(double v) {
    stretch_[2] = v;
  }

  double stretch_g() const {
    return stretch_[1];
  }

  void set_stretch_g(double v) {
    stretch_[1] = v;
  }

  double stretch_b() const {
    return stretch_[0];
  }

  void set_stretch_b(double v) {
    stretch_[0] = v;
  }

  double stretch_a() const {
    return stretch_[3];
  }

  void set_stretch_a(double v) {
    stretch_[3] = v;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stretch_r, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, bias_r, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stretch_g, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, bias_g, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stretch_b, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, bias_b, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stretch_a, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, bias_a, "");
  }

protected:
  cv::Scalar bias_;
  cv::Scalar stretch_;
};

#endif /* __c_scale_channels_routine_h__ */
