/*
 * c_laplacian_map_routine.h
 *
 *  Created on: Sep 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_laplacian_map_routine_h__
#define __c_laplacian_map_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/sharpness_measure/c_laplacian_sharpness_measure.h>

class c_laplacian_map_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_laplacian_map_routine,
      "laplacian_map", "c_laplacian_sharpness_measure map");

  void set_dscale(int v)
  {
    _m.set_dscale(v);
  }

  int dscale() const
  {
    return _m.dscale();
  }

  void set_se_size(const cv::Size & v)
  {
    _m.set_se_size(v);
  }

  const cv::Size & se_size() const
  {
    return _m.se_size();
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_laplacian_sharpness_measure _m;
};

#endif /* __c_laplacian_map_routine_h__ */
