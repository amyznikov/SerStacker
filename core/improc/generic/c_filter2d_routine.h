/*
 * c_filter2d_routine.h
 *
 *  Created on: Mar 21, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_filter2d_routine_h__
#define __c_filter2d_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_filter2d_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_filter2d_routine,
      "filter2D", "Apply cv::filter2D() with user specified kernel");


  void set_kernel(const std::string & v)
  {
    _kernel = v;
    _K.release();
  }

  const std::string & kernel() const
  {
    return _kernel;
  }

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  std::string _kernel;
  cv::Point _anchor = cv::Point(-1,-1);
  double _delta = 0;
  PIXEL_DEPTH _ddepth = PIXEL_DEPTH_NO_CHANGE;
  cv::BorderTypes _borderType = cv::BORDER_DEFAULT;
  cv::Mat1f _K;
};

#endif /* __c_filter2d_routine_h__ */
