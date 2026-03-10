/*
 * c_histogram_normalization_routine.h
 *
 *  Created on: Jun 19, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_histogram_normalization_routine_h__
#define __c_histogram_normalization_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/histogram-tools.h>

class c_histogram_normalization_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_histogram_normalization_routine,
      "histogram_normalization", "histogram normalization");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  COLORID colorid = COLORID_UNKNOWN;
  c_histogram_normalization_options opts;
};

#endif /* __c_histogram_normalization_routine_h__ */
