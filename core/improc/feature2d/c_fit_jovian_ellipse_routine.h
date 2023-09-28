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
    display_gray_image,

    display_detected_planetary_disk_mask,
    display_detected_planetary_disk_edge,

    // display_detected_ellipseAMS,

    display_pca_gx,
    display_pca_gy,

    // display_final_planetary_disk_ellipse,
    display_final_planetary_disk_mask,

    display_final_ellipse_fit,
  };

  void set_display(display_type v);
  display_type display() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  void set_pca_blur(double v);
  double pca_blur() const;

  void set_offset(const cv::Point2f & v);
  const cv::Point2f & offset() const;

  c_jovian_ellipse_detector * detector();
  const c_jovian_ellipse_detector * detector() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  display_type display_type_ = display_final_ellipse_fit;
  c_jovian_ellipse_detector detector_;
};

#endif /* __c_fit_jovian_ellipse_routine_h__ */
