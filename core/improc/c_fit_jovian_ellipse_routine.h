/*
 * c_fit_jovian_ellipse_routine.h
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fit_jovian_ellipse_routine_h__
#define __c_fit_jovian_ellipse_routine_h__

#include "c_image_processor.h"
#include <core/proc/jupiter.h>

class c_fit_jovian_ellipse_routine :
    public c_image_processor_routine
{
public:
  typedef c_fit_jovian_ellipse_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  enum display_type {
    display_uncropped_planetary_disk_mask,
    display_uncropped_planetary_disk_edge,
    display_uncropped_planetary_disk_ellipse,
    display_uncropped_planetary_disk_ellipseAMS,
    display_initial_uncropped_artifial_ellipse,
    display_aligned_uncropped_artifial_ellipse,
    display_uncropped_planetary_disk_ellipseAMS2,
    display_uncropped_planetary_disk_eigen2d_mu,
    display_uncropped_planetary_disk_eigen2d_N,


    display_cropped_gray_image,
    display_cropped_component_mask,
    display_cropped_gradient_image,
    display_cropped_normalized_image,
    display_initial_artificial_ellipse,
    display_initial_ellipse_fit,
    display_final_ellipse_fit,
  };

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("fit_jovian_ellipse", "fit_jovian_ellipse", "detect planetary disk and fit jovian ellipse",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_fit_jovian_ellipse_routine(bool enabled = true);

  static ptr create(bool enabled = true);

  void set_display(display_type v);
  display_type display() const;

  void set_hlines(const std::vector<float> & hlines);
  const std::vector<float> & hlines() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  void set_normalization_scale(int v);
  int normalization_scale() const;

  void set_normalization_blur(double v);
  double normalization_blur() const;

  void set_gradient_blur(double v);
  double gradient_blur() const;

  c_jovian_ellipse_detector * detector();
  const c_jovian_ellipse_detector * detector() const;

  bool serialize(c_config_setting settings) const override;
  bool deserialize(c_config_setting settings) override;

  bool process(cv::InputOutputArray image,
      cv::InputOutputArray mask = cv::noArray()) override;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stdev_factor, "stdev_factor");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, normalization_scale, "normalization_scale");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, normalization_blur, "normalization_blur");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, hlines, "hlines");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, display, "display image type");
  }

protected:
  display_type display_type_ = display_final_ellipse_fit;
  c_jovian_ellipse_detector detector_;
};

#endif /* __c_fit_jovian_ellipse_routine_h__ */
