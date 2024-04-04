/*
 * c_lpg_map_routine.h
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_lpg_map_routine_h__
#define __c_lpg_map_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>


class c_lpg_map_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_lpg_map_routine,
      "lpg_map", "<strong>lap * k + grad</strong><br>Weighted sum of laplacian and gradient modules ");

  void set_k(double v)
  {
    m_.set_k(v);
  }

  double k() const
  {
    return m_.k();
  }

  void set_dscale(int v)
  {
    m_.set_dscale(v);
  }

  int dscale() const
  {
    return m_.dscale();
  }

  void set_uscale(int v)
  {
    m_.set_uscale(v);
  }

  int uscale() const
  {
    return m_.uscale();
  }

  void set_squared(bool v)
  {
    m_.set_squared(v);
  }

  bool squared() const
  {
    return m_.squared();
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
    BIND_PCTRL(ctls, k, "");
    BIND_PCTRL(ctls, dscale, "");
    BIND_PCTRL(ctls, uscale, "");
    BIND_PCTRL(ctls, squared, "");
    BIND_PCTRL(ctls, avgchannel, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, k);
      SERIALIZE_PROPERTY(settings, save, *this, dscale);
      SERIALIZE_PROPERTY(settings, save, *this, uscale);
      SERIALIZE_PROPERTY(settings, save, *this, squared);
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
  c_lpg_sharpness_measure m_;
};

#endif /* __c_lpg_map_routine_h__ */
