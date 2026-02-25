/*
 * c_pyramid_test_routine.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pyramid_test_routine_h__
#define __c_pyramid_test_routine_h__

#include <core/improc/c_image_processor.h>

class c_pyramid_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_pyramid_test_routine,
      "pyramid_test", "c_pyramid_test_routine");

  enum Mode {
    TestPyrDown,
    TestPyrUp,
    TestPyrDownUp,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  static void pyrdown(cv::InputArray src, cv::OutputArray dst,
      const cv::Size & dstsize = cv::Size(),
      int borderType = cv::BORDER_DEFAULT);
  static void pyrup(cv::InputArray src, cv::OutputArray dst,
      const cv::Size & dstsize = cv::Size(),
      int borderType = cv::BORDER_DEFAULT);

protected:
  Mode _mode = TestPyrDown;
};

#endif /* __c_pyramid_test_routine_h__ */
