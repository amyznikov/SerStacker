/*
 * c_desaturate_shadows_routine.h
 *
 *  Created on: May 31, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_desaturate_darks_routine_h__
#define __c_desaturate_darks_routine_h__

#include <core/improc/c_image_processor.h>

class c_desaturate_shadows_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_desaturate_shadows_routine,
      "desaturate_shadows", "Desaturate color on dark image regions (dark sky background)");

  double wmin() const
  {
    return _wmin;
  }
  void set_wmin(double v)
  {
    _wmin = v;
  }

  double wmax() const
  {
    return _wmax;
  }
  void set_wmax(double v)
  {
    _wmax = v;
  }

  double mblur() const
  {
    return _mblur;
  }
  void set_mblur(double v)
  {
    _mblur = v;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  cv::Mat _gray;
  cv::Mat1f _weights;
  double _wmin = 0.1;
  double _wmax = 0.5;
  double _mblur = 1;
};

#endif /* __c_desaturate_darks_routine_h__ */
