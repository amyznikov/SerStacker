/*
 * c_rangeclip_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rangeclip_routine_h__
#define __c_rangeclip_routine_h__

#include "c_image_processor.h"
#include <core/proc/autoclip.h>

class c_rangeclip_routine
    : public c_image_processor_routine
{
public:
  typedef c_rangeclip_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("rangeclip", "rangeclip", "rangeclip",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_rangeclip_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(double min, double max, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_min(double v);
  double min() const;

  void set_max(double v);
  double max() const;


protected:
  double min_ = 0.0;
  double max_ = 1.0;
};

#endif /* __c_rangeclip_routine_h__ */
