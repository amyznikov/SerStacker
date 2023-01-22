/*
 * c_dogsmap_routine.h
 *
 *  Created on: Oct 16, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dogsmap_routine_h__
#define __c_dogsmap_routine_h__

#include "c_image_processor.h"
#include <core/proc/smap.h>

class c_dogsmap_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_dogsmap_routine, "dogsmap", "dogsmap");

  void set_sigma1(double v)
  {
    s1_ = v;
  }

  double sigma1() const
  {
    return s1_;
  }

  void set_sigma2(double v)
  {
    s2_ = v;
  }

  double sigma2() const
  {
    return s2_;
  }

  void set_minv(double v)
  {
    minv_ = v;
  }

  double minv() const
  {
    return minv_;
  }

  void set_scale(int v)
  {
    scale_ = v;
  }

  int scale() const
  {
    return scale_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma1, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma2, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, minv, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scale, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, sigma1);
      SERIALIZE_PROPERTY(settings, save, *this, sigma2);
      SERIALIZE_PROPERTY(settings, save, *this, minv);
      SERIALIZE_PROPERTY(settings, save, *this, scale);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    compute_dogsmap(image, image.getMatRef(), s1_, s2_, scale_, minv_);
    return true;
  }

protected:
  double s1_ = 1;
  double s2_ = 2;
  double minv_ = 1e-9;
  int scale_ = 3;
};

#endif /* __c_dogsmap_routine_h__ */
