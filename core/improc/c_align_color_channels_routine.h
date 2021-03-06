/*
 * c_align_color_channels_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_align_color_channels_routine_h__
#define __c_align_color_channels_routine_h__

#include <core/proc/align_channels.h>
#include "c_image_processor.h"

class c_align_color_channels_routine
    : public c_image_processor_routine
{
public:
  typedef c_align_color_channels_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("align_color_channels", "align_color_channels", "align_color_channels",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_align_color_channels_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(int ecc_reference_channel, ECC_MOTION_TYPE ecc_motion_type, double ecc_eps, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_reference_channel(int v);
  int reference_channel() const;

  void set_enable_threshold(bool v);
  bool enable_threshold() const;

  void set_threshold(double v);
  double threshold() const;

  void set_motion_type(ECC_MOTION_TYPE motion_type);
  ECC_MOTION_TYPE motion_type() const;

  void set_interpolation(enum ECC_INTERPOLATION_METHOD flags);
  enum ECC_INTERPOLATION_METHOD interpolation() const;

  void set_eps(double v);
  double eps() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_smooth_sigma(double v);
  double smooth_sigma() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  c_align_color_channels & algorithm();
  const c_align_color_channels & algorithm() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, reference_channel, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, enable_threshold, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, threshold, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, motion_type, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, interpolation, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, eps, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, max_iterations, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, smooth_sigma, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, update_step_scale, "");
  }

protected:
  c_align_color_channels algorithm_;
  int reference_channel_ = 0;
  bool enable_threshold_ = false;
  double threshold_ = 0;
};

#endif /* __c_align_color_channels_routine_h__ */
