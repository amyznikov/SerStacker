/*
 * c_gradient_routine.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#ifndef __c_gradient_routine_h__
#define __c_gradient_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>
#include <core/proc/gradient.h>

class c_gradient_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gradient_routine,
      "gradient", "compute image gradient");

  enum ComputeMethod
  {
    ComputeMethodFilter1D,
    ComputeMethodSobel,
    ComputeMethodDiagonalGradient,
  };

  enum OutputType {
    OutputGradient,
    OutputGradientX,
    OutputGradientY,
    OutputGradientMagnitude,
    OutputGradientPhase,
    OutputGradientPhaseW,
    OutputGradientPhase90,
    OutputGradientPhase90W,
    // OutputTextureFromGradients,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  ComputeMethod _compute_method = ComputeMethodFilter1D;
  OutputType _output_type = OutputGradientMagnitude;
  PIXEL_DEPTH _ddepth = PIXEL_DEPTH_NO_CHANGE;
  cv::BorderTypes _border_type  = cv::BORDER_DEFAULT;
  double _scale = 1;
  double _delta = 0;
  bool _squared = false;
  bool _erode_mask = false;
};

#endif /* __c_gradient_routine_h__ */
