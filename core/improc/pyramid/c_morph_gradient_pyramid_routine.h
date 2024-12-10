/*
 * c_morph_gradient_pyramid_routine.h
 *
 *  Created on: Sep 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_morph_gradient_pyramid_routine_h__
#define __c_morph_gradient_pyramid_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/morphology.h>

class c_morph_gradient_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_morph_gradient_pyramid_routine,
      "morph_gradient_pyramid", "Display morphological gradient Pyramid layers");

  void set_operation(MORPH_OPERATION v)
  {
    _operation = v;
  }

  MORPH_OPERATION operation() const
  {
    return _operation;
  }

  void set_max_level(int v)
  {
    _max_level = v;
  }

  int max_level() const
  {
    return _max_level;
  }

//  void set_scale_factor(double v)
//  {
//    scale_factor_ = v;
//  }
//
//  double scale_factor() const
//  {
//    return scale_factor_;
//  }

  void set_display_pos(int v)
  {
    _display_pos = v;
  }

  int display_pos() const
  {
    return _display_pos;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  std::vector<cv::Mat> _pyramid;
  //double scale_factor_ = 0.75;
  MORPH_OPERATION _operation = MORPH_LAPLACIAN_ABS;
  int _max_level = 3;
  int _display_pos = 0;
};

#endif /* __c_morph_gradient_pyramid_routine_h__ */
