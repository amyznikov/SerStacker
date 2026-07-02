/*
 * c_fit_saturn_ellipse_routine.h
 *
 *  Created on: Jun 4, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fit_saturn_ellipse_routine_h__
#define __c_fit_saturn_ellipse_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/feature2d/c_saturn_ellipse_detector.h>

class c_fit_saturn_ellipse_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fit_saturn_ellipse_routine,
       "fit_saturn_ellipse", "detect saturn disk and rings and fit ellipses");

  enum display_type {
    display_gray_image,
    display_initial_mask,
    display_pca_mask,
    display_pca_fit,
    display_gradient_image,
    display_gx,
    display_gy,
    display_gr,
    display_grth,
    display_grth_fit,
    display_grid,
    display_final_fit,
    display_debug_image,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  display_type _display_type = display_final_fit;
  c_saturn_ellipse_detector_options _opts;
  c_saturn_ellipse_detector _detector;

  struct c_grid_options {
    double lat_step_deg = 30;
    double lon_step_deg = 30;
  } _grid;

};

#endif /* __c_fit_saturn_ellipse_routine_h__ */
