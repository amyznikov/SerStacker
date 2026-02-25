/*
 * c_harris_map_routine.h
 *
 *  Created on: Jan 25, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_harris_map_routine_h__
#define __c_harris_map_routine_h__

#include <core/proc/sharpness_measure/c_harris_sharpness_measure.h>
#include <core/improc/c_image_processor.h>

class c_harris_map_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_harris_map_routine,
      "hessian_map", "hessian map");

  void set_k(double v)
  {
    _m.set_k(v);
  }

  double k() const
  {
    return _m.k();
  }

  void set_dscale(int v)
  {
    _m.set_dscale(v);
  }

  int dscale() const
  {
    return _m.dscale();
  }

  void set_uscale(int v)
  {
    _m.set_uscale(v);
  }

  int uscale() const
  {
    return _m.uscale();
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
  c_harris_sharpness_measure _m;

};

#endif /* __c_harris_map_routine_h__ */
