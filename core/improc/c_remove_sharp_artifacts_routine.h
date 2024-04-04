/*
 * c_remove_sharp_artifacts_routine.h
 *
 *  Created on: Jul 30, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_remove_sharp_artifacts_routine_h__
#define __c_remove_sharp_artifacts_routine_h__

#include <core/improc/c_image_processor.h>

class c_remove_sharp_artifacts_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_remove_sharp_artifacts_routine,
      "remove_sharp_artifacts",
      "Remove some unsharp mask artifacts");

  void set_erode_radius(const cv::Size & v);
  const cv::Size & erode_radius() const;

  void set_mask_blur_radius(double v);
  double mask_blur_radius() const;

  void set_edge_blur_radius(double v);
  double edge_blur_radius() const;

  void set_fill_holes(bool v);
  bool fill_holes() const;

  void set_noise_scale(double v);
  double noise_scale() const;

  void set_show_mask(bool v);
  bool show_mask() const;

  void set_show_blured_image(bool v);
  bool show_blured_image() const;


  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, erode_radius, "SE radius for erode");
    BIND_PCTRL(ctls, mask_blur_radius, "GaussianBlur sigma");
    BIND_PCTRL(ctls, edge_blur_radius, "GaussianBlur sigma");
    BIND_PCTRL(ctls, fill_holes, "Fill holes inide mask (jovian satellite shadows etc)");
    BIND_PCTRL(ctls, noise_scale, "noise scale");
    BIND_PCTRL(ctls, show_mask, "show objects mask instead of processing");
    BIND_PCTRL(ctls, show_blured_image, "show blured image instead of processing");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, erode_radius);
      SERIALIZE_PROPERTY(settings, save, *this, mask_blur_radius);
      SERIALIZE_PROPERTY(settings, save, *this, edge_blur_radius);
      SERIALIZE_PROPERTY(settings, save, *this, fill_holes);
      SERIALIZE_PROPERTY(settings, save, *this, noise_scale);
      SERIALIZE_PROPERTY(settings, save, *this, show_mask);
      SERIALIZE_PROPERTY(settings, save, *this, show_blured_image);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Size erode_radius_ = cv::Size(5, 5);
  double noise_scale_ = 10;
  double mask_blur_radius_ = 3;
  double edge_blur_radius_ = 1.5;
  bool fill_holes_ = true;
  bool show_mask_ = false;
  bool show_blured_image_ = false;
};

#endif /* __c_remove_sharp_artifacts_routine_h__ */
