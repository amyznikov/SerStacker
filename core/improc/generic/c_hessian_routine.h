/*
 * c_hessian_routine.h
 *
 *  Created on: May 31, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hessian_routine_h__
#define __c_hessian_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>

class c_hessian_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_hessian_routine,
      "hessian", "c_hessian_routine");

  enum OutputType {
    OutputGx,
    OutputGy,
    OutputGxx,
    OutputGyy,
    OutputGxy,
    OutputGxxGyy,
    OutputGxyGxy,
    OutputDet,
    OutputLap,
    OutputDetLap,
    OutputGaussianCurvature,
    OutputMaxEigenValues,
    OutputMinEigenValues,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  OutputType _output_type = OutputGxx;
  cv::BorderTypes _border_type  = cv::BORDER_DEFAULT;
};

#endif /* __c_hessian_routine_h__ */
