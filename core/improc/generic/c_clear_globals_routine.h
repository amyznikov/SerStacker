/*
 * c_clear_globals_routine.h
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_clear_globals_routine_h__
#define __c_clear_globals_routine_h__

#include <core/improc/c_image_processor.h>

class c_clear_globals_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_clear_globals_routine,
      "clear_globals", "c_clear_globals_routine");

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:

};

#endif /* __c_clear_globals_routine_h__ */
