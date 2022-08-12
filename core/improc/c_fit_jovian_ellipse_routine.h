/*
 * c_fit_jovian_ellipse_routine.h
 *
 *  Created on: Aug 12, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fit_jovian_ellipse_routine_h__
#define __c_fit_jovian_ellipse_routine_h__

#include "c_image_processor.h"
#include <core/proc/jupiter.h>

class c_fit_jovian_ellipse_routine :
    public c_image_processor_routine
{
public:
  typedef c_fit_jovian_ellipse_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  enum display_image_type {
    display_gray_image,
    display_component_mask,
    display_gradient_magnitude,
    display_initial_artifical_ellipse,
    display_fitted_artifical_ellipse,
  };

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("fit_jovian_ellipse", "fit_jovian_ellipse", "detect planetary disk and fit jovian ellipse",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_fit_jovian_ellipse_routine(bool enabled = true);

  static ptr create(bool enabled = true);

  void set_display_type(display_image_type v);
  display_image_type display_type() const;

  bool serialize(c_config_setting settings) const override;
  bool deserialize(c_config_setting settings) override;

  bool process(cv::InputOutputArray image,
      cv::InputOutputArray mask = cv::noArray()) override;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, display_type, "display image type");
  }

protected:
  display_image_type display_type_ = display_fitted_artifical_ellipse;
};

#endif /* __c_fit_jovian_ellipse_routine_h__ */
