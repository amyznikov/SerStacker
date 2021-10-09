/*
 * c_auto_correlation_routine.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_auto_correlation_routine_h__
#define __c_auto_correlation_routine_h__

#include "c_image_processor.h"

class c_auto_correlation_routine
    : public c_image_processor_routine
{
public:

  typedef c_auto_correlation_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("auto_correlation", "auto_correlation", "image auto correlation",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_auto_correlation_routine(bool enabled = true);

  static ptr create(bool enabled = true);

  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
};

#endif /* __c_auto_correlation_routine_h__ */
