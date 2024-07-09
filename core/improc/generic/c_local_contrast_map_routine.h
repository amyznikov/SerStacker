/*
 * c_local_contrast_map_routine.h
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_local_contrast_map_routine_h__
#define __c_local_contrast_map_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/sharpness_measure/c_local_contrast_measure.h>

class c_local_contrast_map_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_local_contrast_map_routine,
      "contrast_map", "local contrast map");

  void set_eps(double v)
  {
    m_.set_eps(v);
  }

  double eps() const
  {
    return m_.eps();
  }

  void set_dscale(int v)
  {
    m_.set_dscale(v);
  }

  int dscale() const
  {
    return m_.dscale();
  }

  void set_avgchannel(bool v)
  {
    m_.set_avgchannel(v);
  }

  bool avgchannel() const
  {
    return m_.avgchannel();
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, eps, "");
    BIND_PCTRL(ctls, dscale, "");
    BIND_PCTRL(ctls, avgchannel, "");
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
    m_.create_map(image.getMat(), image);
    return true;
  }

protected:
  c_local_contrast_measure m_;
};

#endif /* __c_local_contrast_map_routine_h__ */
