/*
 * c_polar_warp_routine.h
 *
 *  Created on: Aug 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_polar_warp_routine_h__
#define __c_polar_warp_routine_h__

#include <core/improc/c_image_processor.h>

class c_polar_warp_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_polar_warp_routine,
      "polar_warp", "Apply cv::warpPolar() to image");

  enum INTERPOLATION_MODE
  {
    INTER_NEAREST = cv::INTER_NEAREST,
    INTER_LINEAR = cv::INTER_LINEAR,
    INTER_CUBIC = cv::INTER_CUBIC,
    INTER_AREA = cv::INTER_AREA,
    INTER_LANCZOS4 = cv::INTER_LANCZOS4,
    INTER_LINEAR_EXACT = cv::INTER_LINEAR_EXACT,
#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0) )
    INTER_NEAREST_EXACT = cv::INTER_NEAREST_EXACT,
#endif
  };

  void set_center(const cv::Point2f & v)
  {
    _center = v;
    _rmap.release();
  }

  const cv::Point2f & center() const
  {
    return _center;
  }

  void set_interpolation_mode(INTERPOLATION_MODE v)
  {
    _interpolation = v;
    _rmap.release();
  }

  INTERPOLATION_MODE interpolation_mode() const
  {
    return _interpolation;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);


protected:
  cv::Mat2f _rmap;
  cv::Size _old_src_size;
  cv::Point2f _center;
  INTERPOLATION_MODE _interpolation = INTER_LINEAR;
};

#endif /* __c_polar_warp_routine_h__ */
