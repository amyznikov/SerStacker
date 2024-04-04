/*
 * c_color_diff_routine.h
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_color_diff_routine_h__
#define __c_color_diff_routine_h__

#include <core/improc/c_image_processor.h>

class c_color_diff_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_color_diff_routine,
      "color_diff",
      "Difefrence between color channels and gray");

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    // BIND_PCTRL(ctls, scales, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      // SERIALIZE_PROPERTY(settings, save, *this, scales);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
};

#endif /* __c_color_diff_routine_h__ */
