/*
 * c_local_peak_routine.h
 *
 *  Created on: Mar 16, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_local_peak_routine_h__
#define __c_local_peak_routine_h__

#include <core/improc/c_image_processor.h>

class c_local_peak_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_local_peak_routine,
      "local_peak", "Uses two step filters to find local peaks");

  enum Direction {
    DirectionVert,
    DirectionHorz,
    DirectionBoth,
  };

  enum Output {
    Output1,
    Output2,
    OutputProd
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _peak_radius = 2;
  Direction _direction = DirectionVert;
  Output _output = OutputProd;
  bool _applyMax = false;
};

#endif /* __c_local_peak_routine_h__ */
