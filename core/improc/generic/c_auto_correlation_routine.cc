/*
 * c_auto_correlation_routine.cc
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#include "c_auto_correlation_routine.h"

void c_auto_correlation_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
}

bool c_auto_correlation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  fftComputeAutoCorrelation(image.getMat(), image.getMatRef(), true);
  return true;
}

