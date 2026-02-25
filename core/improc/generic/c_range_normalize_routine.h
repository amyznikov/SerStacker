/*
 * c_range_normalize_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_range_normalize_routine_h__
#define __c_range_normalize_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/normalize.h>

class c_range_normalize_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_range_normalize_routine, "normalize",
      "Apply normalize_image() for given input/output ranges");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double  _input_min = 0.0;
  double  _input_max = 1.0;
  double  _output_min = 0.0;
  double  _output_max = 1.0;
  bool    _auto_input_range = true;
};

#endif /* __c_range_normalize_routine_h__ */
