/*
 * c_local_contrast_map_routine.h
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_local_contrast_map_routine_h__
#define __c_local_contrast_map_routine_h__

#include "c_image_processor.h"
#include <core/proc/focus.h>

class c_local_contrast_map_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_local_contrast_map_routine,
      "contrast_map", "local contrast map");

  void set_eps(double v)
  {
    eps_ = v;
  }

  double eps() const
  {
    return eps_;
  }

  void set_dscale(int v)
  {
    dscale_ = v;
  }

  int dscale() const
  {
    return dscale_;
  }

  void set_avgchannel(bool v)
  {
    avgchannel_ = v;
  }

  bool avgchannel() const
  {
    return avgchannel_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, eps, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, dscale, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, avgchannel, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, eps);
      SERIALIZE_PROPERTY(settings, save, *this, dscale);
      SERIALIZE_PROPERTY(settings, save, *this, avgchannel);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    c_local_contrast_measure::compute_contrast_map(image.getMat(), image, eps_, dscale_, avgchannel_);
    return true;
  }

protected:
  double eps_ = 1e-6;
  int dscale_ = 0;
  bool avgchannel_ = true;
};

#endif /* __c_local_contrast_map_routine_h__ */
