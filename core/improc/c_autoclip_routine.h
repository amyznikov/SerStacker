/*
 * c_autoclip_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_autoclip_routine_h__
#define __c_autoclip_routine_h__

#include "c_image_processor.h"

class c_autoclip_routine
    : public c_image_processor_routine
{
public:
  typedef c_autoclip_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("autoclip", "autoclip", "autoclip",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_autoclip_routine(bool enabled = true);
  static ptr create(bool enabled = true);
  static ptr create(double lclip, double hclip, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_lclip(double v);
  double lclip() const;

  void set_hclip(double v);
  double hclip() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lclip, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, hclip, "");
  }

protected:
  double plo_ = 0.5;
  double phi_ = 99.5;
};


#endif /* __c_autoclip_routine_h__ */
