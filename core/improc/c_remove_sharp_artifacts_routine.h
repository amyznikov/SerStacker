/*
 * c_remove_sharp_artifacts_routine.h
 *
 *  Created on: Jul 30, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_remove_sharp_artifacts_routine_h__
#define __c_remove_sharp_artifacts_routine_h__

#include "c_image_processor.h"

class c_remove_sharp_artifacts_routine:
    public c_image_processor_routine
{
public:
  typedef c_remove_sharp_artifacts_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("remove_sharp_artifacts", "remove some unsharp mask artifacts", "remove some unsharp mask artifacts",
            factory([]() {return ptr(new this_class());}))
    {
    }
  } class_factory;


  c_remove_sharp_artifacts_routine(bool enabled = true);

  static ptr create(bool enabled = true);

  void set_erode_radius(const cv::Size & v);
  const cv::Size & erode_radius() const;

  void set_mask_blur_radius(double v);
  double mask_blur_radius() const;

  void set_edge_blur_radius(double v);
  double edge_blur_radius() const;

  void set_noise_scale(double v);
  double noise_scale() const;

  void set_show_mask(bool v);
  bool show_mask() const;

  void set_show_blured_image(bool v);
  bool show_blured_image() const;

  bool serialize(c_config_setting settings) const override;
  bool deserialize(c_config_setting settings) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, erode_radius, "SE radius for erode");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, mask_blur_radius, "GaussianBlur sigma");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, edge_blur_radius, "GaussianBlur sigma");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, noise_scale, "noise scale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, show_mask, "show objects mask instead of processing");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, show_blured_image, "show blured image instead of processing");
  }

protected:
  cv::Size erode_radius_ = cv::Size(5, 5);
  double noise_scale_ = 10;
  double mask_blur_radius_ = 3;
  double edge_blur_radius_ = 1.5;
  bool show_mask_ = false;
  bool show_blured_image_ = false;
};

#endif /* __c_remove_sharp_artifacts_routine_h__ */
