/*
 * c_image_processor_routine.h
 *
 *  Created on: Jul 15, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_blur_routine_h__
#define __c_median_blur_routine_h__

#include "c_image_processor.h"
#include <opencv2/ximgproc.hpp>

class c_median_blur_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_blur_routine,  "median_blur",
      "Calls <strong>weightedMedianFilter()</strong>");

  void set_radius(int v)
  {
    radius_ = v;
  }

  int radius() const
  {
    return radius_;
  }

  void set_sigma(double v)
  {
    sigma_ = v;
  }

  double sigma() const
  {
    return sigma_;
  }

  void set_weightType(cv::ximgproc::WMFWeightType v)
  {
    weightType_ = v;
  }

  cv::ximgproc::WMFWeightType weightType() const
  {
    return weightType_;
  }

  void set_ignore_mask(bool v)
  {
    ignore_mask_ = v;
  }

  bool ignore_mask() const
  {
    return ignore_mask_;
  }


  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, weightType, "cv::ximgproc::WMFWeightType");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, radius, "Filter radius [px]");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma, "Sigma color");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ignore_mask, "Ignore alpha mask");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, radius);
      SERIALIZE_PROPERTY(settings, save, *this, sigma);
      SERIALIZE_PROPERTY(settings, save, *this, weightType);
      SERIALIZE_PROPERTY(settings, save, *this, ignore_mask);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::ximgproc::WMFWeightType weightType_ = cv::ximgproc::WMF_OFF;
  int radius_ = 1;
  double sigma_ = 25.5 / 255;
  bool ignore_mask_ = true;
};

#endif /* __c_median_blur_routine_h__ */
