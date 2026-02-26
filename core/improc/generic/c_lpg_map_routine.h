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
      "lpg_map", "lap * k + grad<br>Weighted sum of laplacian and gradient modules ");

  void set_k(double v)
  {
    _m.set_k(v);
  }

  double k() const
  {
    return _m.k();
  }

  void set_p(double v)
  {
    _m.set_p(v);
  }

  double p() const
  {
    return _m.p();
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
//  bool copy_parameters_to_clipboard();
//  bool paste_parameters_from_clipboard();
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_lpg_sharpness_measure _m;
};

#endif /* __c_lpg_map_routine_h__ */
