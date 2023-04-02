/*
 * c_unsharp_mask_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_unsharp_mask_routine_h__
#define __c_unsharp_mask_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/unsharp_mask.h>

class c_unsharp_mask_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_unsharp_mask_routine,
      "unsharp_mask", "Apply unsharp mask to image");

  void set_sigma(double v)
  {
    sigma_ = v;
  }

  double sigma() const
  {
    return sigma_;
  }

  void set_alpha(double v)
  {
    alpha_ = v;
  }

  double alpha() const
  {
    return alpha_;
  }

  void set_outmin(double v)
  {
    outmin_ = v;
  }

  double outmin() const
  {
    return outmin_;
  }

  void set_outmax(double v)
  {
    outmax_ = v;
  }

  double outmax() const
  {
    return outmax_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, alpha, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, outmin, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, outmax, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, sigma);
      SERIALIZE_PROPERTY(settings, save, *this, alpha);
      SERIALIZE_PROPERTY(settings, save, *this, outmin);
      SERIALIZE_PROPERTY(settings, save, *this, outmax);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    unsharp_mask(image, mask, image, sigma_, alpha_, outmin_, outmax_);
    return true;
  }

protected:
  double sigma_ = 1, alpha_ = 0.9;
  double outmin_ = -1, outmax_ = -1;
};

#endif /* __c_unsharp_mask_routine_h__ */
