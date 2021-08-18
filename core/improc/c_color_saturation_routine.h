/*
 * c_color_saturation_routine.h
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_color_saturation_routine_h__
#define __c_color_saturation_routine_h__

#include "c_image_processor.h"

class c_color_saturation_routine : public c_image_processor_routine {
public:
  typedef c_color_saturation_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("color_saturation",
            "color saturation",
            "Scale color saturation",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_color_saturation_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(const std::vector<double> & scales, bool enabled = true);

  bool serialize(c_config_setting settings) const override;
  bool deserialize(c_config_setting settings) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_scales(const std::vector<double> & scales);
  const std::vector<double> & scales() const;

protected:
  std::vector<double> scales_;
};

#endif /* __c_color_saturation_routine_h__ */
