/*
 * c_mtf_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mtf_routine_h__
#define __c_mtf_routine_h__

#include <core/mtf/c_pixinsight_mtf.h>
#include <core/improc/c_image_processor.h>

class c_mtf_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_mtf_routine,
      "mtf", "c_pixinsight_mtf");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Vec2d _inputRange = cv::Vec2d(-1,-1);
  cv::Vec2d _outputRange = cv::Vec2d(-1,-1);
  double _shadows = 0;
  double _highlights = 0;
  double _midtones = 0;
  c_pixinsight_mtf _mtf;
};

#endif /* __c_mtf_routine_h__ */
