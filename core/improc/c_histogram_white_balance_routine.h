/*
 * c_histogram_white_balance_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_histogram_white_balance_routine_h__
#define __c_histogram_white_balance_routine_h__

#include "c_image_processor.h"

class c_histogram_white_balance_routine
    : public c_image_processor_routine
{
public:
  typedef c_histogram_white_balance_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("histogram_white_balance", "histogram_white_balance", "histogram_white_balance",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_histogram_white_balance_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(double lclip, double hclip, bool enabled = true);
  bool load(c_config_setting settings) override;
  bool save(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_lclip(double v);
  double lclip() const;

  void set_hclip(double v);
  double hclip() const;

protected:
  double lclip_ = 1;
  double hclip_ = 99;
};

#endif /* __c_histogram_white_balance_routine_h__ */
