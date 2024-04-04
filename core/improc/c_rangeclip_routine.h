/*
 * c_rangeclip_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rangeclip_routine_h__
#define __c_rangeclip_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/autoclip.h>

class c_rangeclip_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_rangeclip_routine,
      "rangeclip", "rangeclip");

  void set_min(double v)
  {
    min_ = v;
  }

  double min() const
  {
    return min_;
  }

  void set_max(double v)
  {
    max_ = v;
  }

  double max() const
  {
    return max_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, min, "");
    BIND_PCTRL(ctls, max, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, min);
      SERIALIZE_PROPERTY(settings, save, *this, max);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    return clip_range(image.getMatRef(), min_, max_/*, mask.getMat()*/);
  }

protected:
  double min_ = 0.0;
  double max_ = 1.0;
};

#endif /* __c_rangeclip_routine_h__ */
