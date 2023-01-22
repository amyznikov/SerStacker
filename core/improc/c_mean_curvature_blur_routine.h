/*
 * c_mean_curvature_blur_routine.h
 *
 *  Created on: Aug 1, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mean_curvature_blur_routine_h__
#define __c_mean_curvature_blur_routine_h__

#include "c_image_processor.h"
#include <core/proc/mean_curvature_blur.h>

class c_mean_curvature_blur_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_mean_curvature_blur_routine,
      "mean_curvature_blur", " mean curvature blur filter");


  void set_iterations(int v)
  {
    iterations_ = v;
  }

  int iterations() const
  {
    return iterations_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, iterations, "number of iterations");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, iterations);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if ( iterations_ > 0 ) {
      mean_curvature_blur(image.getMat(), image.getMatRef(), iterations_);
    }
    return true;
  }

protected:
  int iterations_ = 1;
};

#endif /* __c_mean_curvature_blur_routine_h__ */
