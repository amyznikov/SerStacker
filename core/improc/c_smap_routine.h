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
  static ptr create(int lksize, int scale_size, double minv, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_lksize(int );
  int lksize() const;

  void set_scale_size(int);
  int scale_size() const;

  void set_minv(double);
  double minv() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lksize, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scale_size, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, minv, "");
  }

protected:
  int lksize_ = 7;
  int scale_size_ = 6;
  double minv_ = 1;
};



#endif /* __c_smap_routine_h__ */
