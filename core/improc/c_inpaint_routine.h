/*
 * c_inpaint_routine.h
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_inpaint_routine_h__
#define __c_inpaint_routine_h__

#include "c_image_processor.h"

class c_inpaint_routine
    : public c_image_processor_routine
{
public:
  typedef c_inpaint_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("inpaint", "inpaint", "inpaint missing pixels in image",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_inpaint_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;
};

#endif /* __c_inpaint_routine_h__ */
