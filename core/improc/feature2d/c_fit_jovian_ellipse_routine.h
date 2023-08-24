/*
 * c_fit_jovian_ellipse_routine.h
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fit_jovian_ellipse_routine_h__
#define __c_fit_jovian_ellipse_routine_h__

#include <core/proc/image_registration/c_jovian_derotation.h>
#include <core/improc/c_image_processor.h>

class c_fit_jovian_ellipse_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fit_jovian_ellipse_routine,
       "fit_jovian_ellipse", "detect planetary disk and fit jovian ellipse");

  enum display_type {
    display_detected_planetary_disk_mask,
    display_detected_planetary_disk_edge,
    display_planetary_disk_ellipse_edge,
    display_detected_ellipseAMS,
    display_initial_artifial_ellipse_edge,
    display_remapped_artifial_ellipse_edge,
    display_aligned_artifial_ellipse_edge,
    display_aligned_artifial_ellipse_mask,
    display_planetary_disk_ellipseAMS2,
    display_gray_image,
    display_final_ellipse_fit,
    display_gradient_test_image,
  };

  void set_display(display_type v);
  display_type display() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  c_jovian_ellipse_detector * detector();
  const c_jovian_ellipse_detector * detector() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, stdev_factor, "stdev_factor");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, display, "display image type");
  }

  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  display_type display_type_ = display_final_ellipse_fit;
  c_jovian_ellipse_detector detector_;
};

#endif /* __c_fit_jovian_ellipse_routine_h__ */
