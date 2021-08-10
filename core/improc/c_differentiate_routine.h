/*
 * c_differentiate_routine.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#ifndef __c_differentiate_routine_h__
#define __c_differentiate_routine_h__

#include "c_image_processor.h"

class c_differentiate_routine
    : public c_image_processor_routine
{
public:
  typedef c_differentiate_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("anscombe", "anscombe transform", "anscombe transform",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_differentiate_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;
};

#endif /* __c_differentiate_routine_h__ */
