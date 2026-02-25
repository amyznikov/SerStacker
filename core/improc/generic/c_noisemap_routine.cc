/*
 * c_noisemap_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_noisemap_routine.h"

void c_noisemap_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
}

bool c_noisemap_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  create_noise_map(image.getMatRef(), image.getMatRef(), mask);
  return true;
}

