/*
 * c_ridgeness_routine.h
 *
 *  Created on: May 1, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_ridgeness_routine_h__
#define __c_ridgeness_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/cdiffs.h>

class c_ridgeness_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_ridgeness_routine,
      "ridgeness", "compute image ridgeness");

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
};

#endif /* __c_ridgeness_routine_h__ */
