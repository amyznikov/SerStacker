/*
 * c_anscombe_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_anscombe_routine_h__
#define __c_anscombe_routine_h__

#include "c_image_processor.h"
#include <core/proc/c_anscombe_transform.h>


class c_anscombe_routine
    : public c_image_processor_routine
{
public:
  typedef c_anscombe_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("anscombe", "anscombe transform", "anscombe transform",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_anscombe_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(enum anscombe_method m, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_method(enum anscombe_method v);
  enum anscombe_method method() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, method, "");
  }

protected:
  c_anscombe_transform anscombe_;
};



#endif /* __c_anscombe_routine_h__ */
