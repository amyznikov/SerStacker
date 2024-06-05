/*
 * c_wmf_routine.h
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_wmf_routine_h__
#define __c_wmf_routine_h__

#include <core/improc/c_image_processor.h>
#include <opencv2/ximgproc.hpp>

class c_wmf_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_wmf_routine,
      "wmf",
      "calls cv::ximgproc::weightedMedianFilter()");

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


  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, weightType, "cv::ximgproc::WMFWeightType");
    BIND_PCTRL(ctls, radius, "Filter radius [px]");
    BIND_PCTRL(ctls, sigma, "Sigma color");
    BIND_PCTRL(ctls, ignore_mask, "Ignore alpha mask");
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

#endif /* __c_wmf_routine_h__ */
