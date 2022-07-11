/*
 * c_pyrdown_routine.h
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pyrdown_routine_h__
#define __c_pyrdown_routine_h__

#include "c_image_processor.h"

class c_pyrdown_routine:
    public c_image_processor_routine
{
public:
  typedef c_pyrdown_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("pyrdown", "pyrdown", "pyrdown",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_pyrdown_routine(int count = 1 , bool enabled = true);

  static ptr create(int count = 1, bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_count(int v);
  int count() const;

  void set_borderType(cv::BorderTypes v);
  cv::BorderTypes borderType() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, count, "count of times for pyDown (negative value for pyrUp instead)");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, borderType, "enum cv::BorderTypes");
  }

protected:
  int count_;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;
};

#endif /* __c_pyrdown_routine_h__ */
