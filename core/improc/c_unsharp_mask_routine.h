/*
 * c_unsharp_mask_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_unsharp_mask_routine_h__
#define __c_unsharp_mask_routine_h__

#include "c_image_processor.h"

class c_unsharp_mask_routine
: public c_image_processor_routine
{
public:
  typedef c_unsharp_mask_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("unsharp_mask", "unsharp mask", "unsharp mask",
            factory([]() {return ptr(new this_class());}))
    {
    }
  } class_factory;

  c_unsharp_mask_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(double sigma, double alpha = 0.9, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  double sigma() const;
  void set_sigma(double v);

  double alpha() const;
  void set_alpha(double v);

protected:
  double sigma_ = 1, alpha_ = 0.9;
  double outmin_ = -1, outmax_ = -1;
};

#endif /* __c_unsharp_mask_routine_h__ */
