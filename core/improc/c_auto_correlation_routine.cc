/*
 * c_auto_correlation_routine.cc
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#include "c_auto_correlation_routine.h"
#include <core/proc/fft.h>
#include <core/debug.h>

c_auto_correlation_routine::c_class_factory c_auto_correlation_routine::class_factory;

c_auto_correlation_routine::c_auto_correlation_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_auto_correlation_routine::ptr c_auto_correlation_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_auto_correlation_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }
  return true;
}

bool c_auto_correlation_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }
  return true;
}

bool c_auto_correlation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  fftComputeAutoCorrelation(image.getMat(), image.getMatRef());
  return true;
}
