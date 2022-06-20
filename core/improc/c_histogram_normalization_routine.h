/*
 * c_histogram_normalization_routine.h
 *
 *  Created on: Jun 19, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_histogram_normalization_routine_h__
#define __c_histogram_normalization_routine_h__

#include "c_image_processor.h"

class c_histogram_normalization_routine:
    public c_image_processor_routine
{
public:
  typedef c_histogram_normalization_routine this_class;
  typedef c_image_processor_routine base;
  typedef std::shared_ptr<this_class> ptr;

  enum histogram_normalization_type {
    normalize_mean,
    //normalize_median,
  };


  static struct c_class_factory : public base::class_factory {
    c_class_factory() :
        base::class_factory("histogram_normalization", "histogram_normalization", "histogram_normalization",
            factory([]() {return ptr(new this_class());})) {}
  } class_factory;


  c_histogram_normalization_routine(bool enabled = true);

  static ptr create(bool enabled = true);
  bool deserialize(c_config_setting settings) override;
  bool serialize(c_config_setting settings) const override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  void set_normalization_type(histogram_normalization_type v);
  histogram_normalization_type normalization_type() const;

  void set_offset(const cv::Scalar & v);
  const cv::Scalar & offset() const;

  void set_scale(const cv::Scalar & v);
  const cv::Scalar & scale() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, normalization_type, "normalization_type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, offset, "offset");
    //ADD_IMAGE_PROCESSOR_CTRL(ctls, scale, "scale");
  }

protected:
  histogram_normalization_type normalization_type_ = normalize_mean;
  cv::Scalar offset_ = cv::Scalar::all(0);
  cv::Scalar scale_ = cv::Scalar::all(1);

};

#endif /* __c_histogram_normalization_routine_h__ */
