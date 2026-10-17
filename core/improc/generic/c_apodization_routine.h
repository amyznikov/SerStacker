/*
 * c_apodization_routine.h
 *
 *  Created on: Oct 12, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_apodization_routine_h__
#define __c_apodization_routine_h__

#include <core/improc/c_image_processor.h>

class c_apodization_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_apodization_routine,
      "apodization", "Apply apodization window to image");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  cv::Mat1f apodizationWindow;
  double edgeSize = 15;
  double oldEdgeSize = 15;
  bool zeroMean = true;
};

#endif /* __c_apodization_routine_h__ */
