/*
 * c_color_saturation_routine.h
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_color_saturation_routine_h__
#define __c_color_saturation_routine_h__

#include "c_image_processor.h"


class c_color_saturation_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_color_saturation_routine,
      "color_saturation",
      "Multi-Scale color saturation.<br>"
      "Based on gimp_rgb_to_hsl() and OpenCV image pyramid.");


  void set_scales(const std::vector<double> & scales)
  {
    scales_ = scales;
  }

  const std::vector<double> & scales() const
  {
    return scales_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scales, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, scales);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  std::vector<double> scales_;
};

#endif /* __c_color_saturation_routine_h__ */
