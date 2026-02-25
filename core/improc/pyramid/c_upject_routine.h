/*
 * c_upject_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_upject_routine_h__
#define __c_upject_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/downstrike.h>

class c_upject_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_upject_routine,
      "upject",
      "2x upsampling step by injecting EVEN or UNEVEN rows and columns");

  enum UpjectMode {
    UpjectUneven,
    UpjectEven,
  };

  enum FillMode {
    FillZero,
    FillAvg,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  UpjectMode _mode = UpjectUneven;
  FillMode _fill_mode = FillAvg;
  cv::Size _dstSize = cv::Size (-1,-1);
};

#endif /* __c_upject_routine_h__ */
