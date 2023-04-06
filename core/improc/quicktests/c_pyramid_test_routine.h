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

  void set_mode(Mode v)
  {
    mode_ = v;
  }

  Mode mode() const
  {
    return mode_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, mode, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, mode);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  static void pyrdown(cv::InputArray src, cv::OutputArray dst,
      const cv::Size & dstsize = cv::Size(),
      int borderType = cv::BORDER_DEFAULT);
  static void pyrup(cv::InputArray src, cv::OutputArray dst,
      const cv::Size & dstsize = cv::Size(),
      int borderType = cv::BORDER_DEFAULT);

protected:
  Mode mode_ = TestPyrDown;
};

#endif /* __c_pyramid_test_routine_h__ */
