/*
 * c_gaussian_filter_routine.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#ifndef __c_gaussian_filter_routine_h__
#define __c_gaussian_filter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_gaussian_filter.h>

class c_gaussian_blur_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gaussian_blur_routine,
      "gaussian_blur", "Gaussian Blur");

  enum StereoMode {
    StereoNone,
    StereoHLayout,
    StereoVLayout,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  double _sigmax = 1;
  double _sigmay = -1;
  int _ksizex = -1;
  int _ksizey = -1;
  StereoMode _stereo_mode = StereoNone;
  cv::BorderTypes _border_type = cv::BORDER_REFLECT101;
  //  cv::Scalar _border_value = cv::Scalar::all(0);
};

#endif /* __c_gaussian_filter_routine_h__ */
