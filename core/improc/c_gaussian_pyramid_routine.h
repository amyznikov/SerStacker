/*
 * c_gaussian_pyramid_routine.h
 *
 *  Created on: Jul 13, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_gaussian_pyramid_routine_h__
#define __c_gaussian_pyramid_routine_h__

#include "c_image_processor.h"

class c_gaussian_pyramid_routine:
    public c_image_processor_routine
{
public:
  typedef c_gaussian_pyramid_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("gaussian_pyramid",
            "gaussian pyramid",
            "Scale gaussian pyramid layers",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_gaussian_pyramid_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  static ptr create(const std::vector<double> & scales, bool enabled = true);

  bool serialize(c_config_setting settings) const override;
  bool deserialize(c_config_setting settings) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_scales(const std::vector<double> & scales);
  const std::vector<double> & scales() const;

  void set_borderType(cv::BorderTypes v);
  cv::BorderTypes borderType() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scales, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, borderType, "");
  }

protected:
  std::vector<double> scales_;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;

};

#endif /* __c_gaussian_pyramid_routine_h__ */
