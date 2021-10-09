/*
 * c_gaussian_filter_routine.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#ifndef __c_gaussian_filter_routine_h__
#define __c_gaussian_filter_routine_h__

#include "c_image_processor.h"

class c_gaussian_filter_routine:
    public c_image_processor_routine
{
public:

  typedef c_gaussian_filter_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("gaussian_filter", "gaussian_filter", "gaussian filter",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_gaussian_filter_routine(bool enabled = true);
  static ptr create(bool enabled = true);
  static ptr create(double sigma, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  double sigma() const;
  void set_sigma(double v);

protected:
  double sigma_ = 1;
};

#endif /* __c_gaussian_filter_routine_h__ */
