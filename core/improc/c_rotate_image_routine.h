/*
 * c_rotate_image_routine.h
 *
 *  Created on: Jun 10, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rotate_image_routine_h__
#define __c_rotate_image_routine_h__

#include "c_image_processor.h"

class c_rotate_image_routine:
    public c_image_processor_routine
{
public:
  typedef c_rotate_image_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("rotate", "rotate image", "rotate image",
            factory([]() {return ptr(new this_class());}))
    {
    }
  } class_factory;


  c_rotate_image_routine(bool enabled = true);

  static ptr create(bool enabled = true);

  void set_rotation_angle(double degrees);
  double rotation_angle() const;


  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, rotation_angle, "rotation angle in degrees");
  }

protected:
  double rotation_angle_ = 0 ;
};

#endif /* __c_rotate_image_routine_h__ */
