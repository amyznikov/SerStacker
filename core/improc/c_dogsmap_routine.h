/*
 * c_dogsmap_routine.h
 *
 *  Created on: Oct 16, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dogsmap_routine_h__
#define __c_dogsmap_routine_h__

#include "c_image_processor.h"

class c_dogsmap_routine:
    public c_image_processor_routine
{
public:
  typedef c_dogsmap_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("dogsmap", "dogsmap", "dogsmap",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_dogsmap_routine(bool enabled = true);
  c_dogsmap_routine(double s1, double s2, int scale, double minv, bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(double s1, double s2, int scale, double minv, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_sigma1(double);
  double sigma1() const;

  void set_sigma2(double);
  double sigma2() const;

  void set_minv(double);
  double minv() const;

  void set_scale(int );
  int scale() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma1, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma2, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, minv, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scale, "");
  }

protected:
  double s1_ = 1;
  double s2_ = 2;
  double minv_ = 1e-9;
  int scale_ = 3;
};

#endif /* __c_dogsmap_routine_h__ */
