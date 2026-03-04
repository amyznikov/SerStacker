/*
 * c_adaptive_gaussian_blur_routine.h
 *
 *  Created on: Mar 2, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_adaptive_blend_filter_routine_h__
#define __c_adaptive_blend_filter_routine_h__

#include <core/improc/c_image_processor.h>

class c_adaptive_gaussian_blur_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_adaptive_gaussian_blur_routine,
      "c_adaptive_gaussian_blur_routine", "Apply adaptive denoise filter");

  enum DisplayType {
    DisplayFiltered,
    DisplayRamp,
    DisplayTex,
    DisplayEdges,
    DisplayLpass,
    DisplayHpass,
    DisplayWeights,
    DisplayDetail,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Mat ramp;
  cv::Mat tex;
  cv::Mat edges;
  cv::Mat lpass;
  cv::Mat hpass;
  cv::Mat weights;
  cv::Mat detail;

  int seRadius = 2;
  int mkRadius = 2;
  double edgeLimit = 99;
  double gfSigma = 1.5;
  double hBoost = 1;
  float x1 = 0.1f;
  float a1 = 0.2f;
  float x2 = 0.9f;
  float a2 = 0.5f;

  DisplayType displayType = DisplayFiltered;
};

#endif /* __c_adaptive_blend_filter_routine_h__ */
