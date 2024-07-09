/*
 * c_histogram_white_balance_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_histogram_white_balance_routine_h__
#define __c_histogram_white_balance_routine_h__

#include <core/improc/c_image_processor.h>

class c_histogram_white_balance_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_histogram_white_balance_routine,
      "histogram_white_balance",
      "GrayWorld-like image white balance based on image histogram stretch");

  void set_lclip(double v)
  {
    lclip_ = v;
  }

  double lclip() const
  {
    return lclip_;
  }

  void set_hclip(double v)
  {
    hclip_ = v;
  }

  double hclip() const
  {
    return hclip_;
  }

  void set_threshold(double v)
  {
    threshold_ = v;
  }

  double threshold() const
  {
    return threshold_;
  }

  void set_enable_threshold(bool v)
  {
    enable_threshold_ = v;
  }

  bool enable_threshold() const
  {
    return enable_threshold_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, lclip, "");
    BIND_PCTRL(ctls, hclip, "");
    BIND_PCTRL(ctls, enable_threshold, "");
    BIND_PCTRL(ctls, threshold, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, lclip);
      SERIALIZE_PROPERTY(settings, save, *this, hclip);
      SERIALIZE_PROPERTY(settings, save, *this, enable_threshold);
      SERIALIZE_PROPERTY(settings, save, *this, threshold);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  double lclip_ = 1;
  double hclip_ = 99;
  double threshold_ = 0;
  bool enable_threshold_ = false;
};

#endif /* __c_histogram_white_balance_routine_h__ */
