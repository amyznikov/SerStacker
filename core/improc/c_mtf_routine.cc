/*
 * c_mtf_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_mtf_routine.h"

c_mtf_routine::c_class_factory c_mtf_routine::class_factory;

c_mtf_routine::c_mtf_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_mtf_routine::ptr c_mtf_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

bool c_mtf_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return mtf_->apply(image, image);
}

const c_pixinsight_midtones_transfer_function::ptr & c_mtf_routine::mtf() const
{
  return mtf_;
}

bool c_mtf_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  return true;
}

bool c_mtf_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  return true;
}

