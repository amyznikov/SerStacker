/*
 * c_flip_image_routine.h
 *
 *  Created on: Nov 22, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_flip_image_routine_h__
#define __c_flip_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_flip_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_flip_image_routine,
       "flip", "Call <strong>cv::flip()</strong> on image");

  void set_hflip(bool v)
  {
    hflip_ = v;
  }

  bool hflip() const
  {
    return hflip_;
  }

  void set_vflip(bool v)
  {
    vflip_ = v;
  }

  bool vflip() const
  {
    return vflip_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  bool hflip_ = true;
  bool vflip_ = false;
};

#endif /* __c_flip_image_routine_h__ */
