/*
 * c_smap_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_smap_routine_h__
#define __c_smap_routine_h__

#include "c_image_processor.h"

class c_smap_routine
    : public c_image_processor_routine
{
public:
  typedef c_smap_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("smap", "smap", "smap",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_smap_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(double minv, double scale = 1.0 / 16, bool enabled = true);
  bool load(c_config_setting settings) override;
  bool save(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_minv(double v);
  double minv() const;

  void set_scale(double v);
  double scale() const;

protected:
  double minv_ = 0;
  double scale_ = 1.0/16;
};



#endif /* __c_smap_routine_h__ */
