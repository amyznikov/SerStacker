/*
 * c_fit_jovian_ellipse_routine.h
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fit_jovian_ellipse_routine_h__
#define __c_fit_jovian_ellipse_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/feature2d/c_jovian_ellipse_detector.h>

class c_fit_jovian_ellipse_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_fit_jovian_ellipse_routine,
       "fit_jovian_ellipse", "detect planetary disk and fit jovian ellipse");

  enum display_type {
    display_gray_image,
    display_normalized_image,
    display_gx,
    display_gy,
    display_g,
    display_gr,
    display_grth,
    display_grthc,
    display_detected_planetary_disk_mask,
    display_detected_planetary_disk_edge,
    display_final_planetary_disk_mask,
    display_final_ellipse_fit,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  display_type _display_type = display_final_ellipse_fit;
  c_jovian_ellipse_detector_options _opts;
  c_jovian_ellipse_detector _detector;
};

#endif /* __c_fit_jovian_ellipse_routine_h__ */
