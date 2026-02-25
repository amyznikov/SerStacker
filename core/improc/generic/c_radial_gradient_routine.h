/*
 * c_radial_gradient_routine.h
 *
 *  Created on: Sep 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_radial_gradient_routine_h__
#define __c_radial_gradient_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>
#include <core/proc/gradient.h>

class c_radial_gradient_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_radial_gradient_routine,
      "radial_gradient", "compute radial gradient on image");

  enum OutputType {
    OutputRadialGradient,
    OutputTangentialGradient,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  OutputType _output_type = OutputRadialGradient;
  cv::Point2f _reference_point = cv::Point2f(0,0);
  int _kradius = 3;
  double _scale = 1;
  double _delta = 0;
  bool _magnitude = false;
  bool _squared = false;
  bool _erode_mask = false;
};

#endif /* __c_radial_gradient_routine_h__ */
