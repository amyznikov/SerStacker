/*
 * c_image_processor_routine.h
 *
 *  Created on: Jul 15, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_blur_routine_h__
#define __c_median_blur_routine_h__

#include "c_image_processor.h"
#include <opencv2/ximgproc.hpp>

class c_median_blur_routine:
    public c_image_processor_routine
{
public:
  typedef c_median_blur_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("median_blur", "median blur", "weighted median blur",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;

  c_median_blur_routine(bool enabled = true);
  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_radius(int v);
  int radius() const;

  void set_sigma(double v);
  double sigma() const;

  void set_weightType(cv::ximgproc::WMFWeightType);
  cv::ximgproc::WMFWeightType weightType() const;

  void set_ignore_mask(bool v);
  bool ignore_mask() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, weightType, "cv::ximgproc::WMFWeightType");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, radius, "Filter radius [px]");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma, "Sigma color");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ignore_mask, "Ignore alpha mask");
  }

protected:
  cv::ximgproc::WMFWeightType weightType_ = cv::ximgproc::WMF_OFF;
  int radius_ = 1;
  double sigma_ = 25.5 / 255;
  bool ignore_mask_ = true;
};

#endif /* __c_median_blur_routine_h__ */
