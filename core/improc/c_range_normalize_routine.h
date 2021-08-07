/*
 * c_range_normalize_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_range_normalize_routine_h__
#define __c_range_normalize_routine_h__

#include "c_image_processor.h"

class c_range_normalize_routine
    : public c_image_processor_routine
{
public:
  typedef c_range_normalize_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("normalize", "normalize", "normalize",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_range_normalize_routine(bool enabled = true);
  static ptr create(bool enabled = true);
  static ptr create(double outmin, double outmax, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_auto_input_range(bool v);
  bool auto_input_range() const;

  void set_input_min(double  v);
  double input_min() const;

  void set_input_max(double  v);
  double input_max() const;

  void set_output_min(double v);
  double output_min() const;

  void set_output_max(double v);
  double output_max() const;

protected:
  double  input_min_ = 0.0;
  double  input_max_ = 1.0;
  double  output_min_ = 0.0;
  double  output_max_ = 1.0;
  bool    auto_input_range_ = true;
};

#endif /* __c_range_normalize_routine_h__ */
