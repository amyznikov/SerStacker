/*
 * c_lpg_map_routine.h
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_lpg_map_routine_h__
#define __c_lpg_map_routine_h__

#include "c_image_processor.h"
#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>


class c_lpg_map_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_lpg_map_routine,
      "lpg_map", "weighted sum of laplacian and gradient modules");

  void set_laplacian_weight(double v)
  {
    laplacian_weight_  =v;
  }

  double laplacian_weight() const
  {
    return laplacian_weight_;
  }

  void set_gradient_weight(double v)
  {
    gradient_weight_  =v;
  }

  double gradient_weight() const
  {
    return gradient_weight_;
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
    ADD_IMAGE_PROCESSOR_CTRL(ctls, laplacian_weight, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, gradient_weight, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, dscale, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, avgchannel, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, laplacian_weight);
      SERIALIZE_PROPERTY(settings, save, *this, gradient_weight);
      SERIALIZE_PROPERTY(settings, save, *this, dscale);
      SERIALIZE_PROPERTY(settings, save, *this, avgchannel);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    c_lpg_sharpness_measure::compute_lpg_map(image.getMat(), image,
        laplacian_weight_, gradient_weight_, dscale_, avgchannel_);

    return true;
  }

protected:
  double laplacian_weight_ = 1;
  double gradient_weight_ = 1;
  int dscale_ = 0;
  bool avgchannel_ = true;

};

#endif /* __c_lpg_map_routine_h__ */
