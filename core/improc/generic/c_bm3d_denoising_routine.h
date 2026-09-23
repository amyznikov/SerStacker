/*
 * c_bm3d_denoising_routine.h
 *
 *  Created on: Sep 23, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_bm3d_denoising_routine_h__
#define __c_bm3d_denoising_routine_h__

#include <core/improc/c_image_processor.h>

class c_bm3d_denoising_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_bm3d_denoising_routine,
      "bm3d_denoising", "Apply cv::xphoto::bm3dDenoising()");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  float h = 1;
  int templateWindowSize = 4;
  int searchWindowSize = 16;
  int blockMatchingStep1 = 2500;
  int blockMatchingStep2 = 400;
  int groupSize = 8;
  int slidingStep = 1;
  float beta = 2.0f;
  cv::NormTypes normType = cv::NORM_L2;
  bool luminanceOnly = false;
  // cv::xphoto::Bm3dSteps step = cv::xphoto::BM3D_STEPALL;
  // int transformType = cv::xphoto::HAAR; Currently only Haar transform is supported
};

#endif /* __c_bm3d_denoising_routine_h__ */
