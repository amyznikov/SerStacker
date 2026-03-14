/*
 * c_lunar_birdview_routine.h
 *
 *  Created on: Mar 14, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_lunar_birdview_routine_h__
#define __c_lunar_birdview_routine_h__

#include <core/improc/c_image_processor.h>

class c_lunar_birdview_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_lunar_birdview_routine,
      "lunar_birdview", "homography transform to moon closeup image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _lat = 43.0; // [deg]
  double _lon = 11.0; // [deg]
  double _rotation = 0; // [deg]
  cv::InterpolationFlags  _interpolation = cv::INTER_LINEAR;
  cv::BorderTypes _borderMode = cv::BORDER_CONSTANT;
  cv::Scalar _borderValue;

};

#endif /* __c_lunar_birdview_routine_h__ */
