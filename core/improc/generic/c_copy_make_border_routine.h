/*
 * c_copy_make_border_routine.h
 *
 *  Created on: Jan 24, 2026
 *      Author: gandriim
 */

#ifndef __c_copy_make_border_routine_h__
#define __c_copy_make_border_routine_h__

#include <core/improc/c_image_processor.h>

class c_copy_make_border_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_copy_make_border_routine,
      "copy_make_border", "cv::copyMakeBorder()");

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  cv::BorderTypes _borderType = cv::BORDER_REFLECT101;
  cv::Scalar _borderValue = cv::Scalar::all(0);
  int _left = 0;
  int _right = 0;
  int _top = 0;
  int _bottom = 0;
};

#endif /* __c_copy_make_border_routine_h__ */
