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
  bool load(c_config_setting settings) override;
  bool save(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_outmin(double v);
  double outmin() const;

  void set_outmax(double v);
  double outmax() const;

protected:
  double outmin_ = 0.0;
  double outmax_ = 1.0;
};

#endif /* __c_range_normalize_routine_h__ */
