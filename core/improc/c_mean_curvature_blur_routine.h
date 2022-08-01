/*
 * c_mean_curvature_blur_routine.h
 *
 *  Created on: Aug 1, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_mean_curvature_blur_routine_h__
#define __c_mean_curvature_blur_routine_h__

#include "c_image_processor.h"

class c_mean_curvature_blur_routine:
    public c_image_processor_routine
{
public:
  typedef c_mean_curvature_blur_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("mean_curvature_blur", "mean-curvature-blur", "mean-curvature-blur",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_mean_curvature_blur_routine(bool enabled = true);
  c_mean_curvature_blur_routine(int iterations, bool enabled = true);
  static ptr create(bool enabled = true);
  static ptr create(int iterations, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_iterations(int v);
  int iterations() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, iterations, "number of iterations");
  }

protected:
  int iterations_ = 1;
};

#endif /* __c_mean_curvature_blur_routine_h__ */
