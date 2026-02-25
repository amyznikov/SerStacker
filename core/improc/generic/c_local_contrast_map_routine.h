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
    _m.set_eps(v);
  }

  double eps() const
  {
    return _m.eps();
  }

  void set_dscale(int v)
  {
    _m.set_dscale(v);
  }

  int dscale() const
  {
    return _m.dscale();
  }

  void set_avgchannel(bool v)
  {
    _m.set_avgchannel(v);
  }

  bool avgchannel() const
  {
    return _m.avgchannel();
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_local_contrast_measure _m;
};

#endif /* __c_local_contrast_map_routine_h__ */
