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
#include <core/proc/white_balance/histogram_normalization.h>
//#include <core/io/debayer.h>

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
  COLORID _colorid = COLORID_UNKNOWN;
  histogram_normalization_type _normalization_type = histogram_normalize_mean;
  cv::Scalar _offset = cv::Scalar::all(0);
  cv::Scalar _stretch = cv::Scalar::all(1);
};

#endif /* __c_histogram_normalization_routine_h__ */
