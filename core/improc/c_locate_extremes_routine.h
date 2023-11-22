/*
 * c_locate_extremes_routine.h
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_locate_extremes_routine_h__
#define __c_locate_extremes_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/locate_extremes.h>


class c_locate_extremes_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_locate_extremes_routine,
      "locate_extremes", "c_locate_extremes_routine");

  enum OUTPUT_CHANNEL {
    OUTPUT_IMAGE,
    OUTPUT_MASK
  };

  // Supported border types only
  enum BorderType
  {
    BORDER_CONSTANT = cv::BORDER_CONSTANT,
    BORDER_REPLICATE = cv::BORDER_REPLICATE,
    BORDER_REFLECT = cv::BORDER_REFLECT,
    BORDER_REFLECT_101 = cv::BORDER_REFLECT_101,
    BORDER_TRANSPARENT = cv::BORDER_TRANSPARENT,
  };

  void set_se_shape(cv::MorphShapes v)
  {
    opts_.se_shape = v;
  }

  cv::MorphShapes se_shape() const
  {
    return opts_.se_shape;
  }

  void set_se_size(const cv::Size & v)
  {
    opts_.se_size = v;
  }

  const cv::Size& se_size() const
  {
    return opts_.se_size;
  }

  void set_anchor(const cv::Point & v)
  {
    opts_.anchor = v;
  }

  const cv::Point & anchor() const
  {
    return opts_.anchor;
  }

  void set_border_type(BorderType v)
  {
    opts_.border_type = static_cast<cv::BorderTypes>(v);
  }

  BorderType border_type() const
  {
    return static_cast<BorderType>(opts_.border_type);
  }

  void set_border_value(const cv::Scalar & v)
  {
    opts_.border_value = v;
  }

  const cv::Scalar & border_value() const
  {
    return opts_.border_value;
  }

  void set_locate_maximums(bool v)
  {
    opts_.locate_maximums = v;
  }

  bool locate_maximums() const
  {
    return opts_.locate_maximums;
  }

  void set_locate_minimums(bool v)
  {
    opts_.locate_minimums = v;
  }

  bool locate_minimums() const
  {
    return opts_.locate_minimums;
  }

  void set_maximums_alpha(double v)
  {
    opts_.maximums_alpha = v;
  }

  double maximums_alpha() const
  {
    return opts_.maximums_alpha;
  }

  void set_maximums_beta(double v)
  {
    opts_.maximums_beta = v;
  }

  double maximums_beta() const
  {
    return opts_.maximums_beta;
  }

  void set_minimums_alpha(double v)
  {
    opts_.minimums_alpha = v;
  }

  double minimums_alpha() const
  {
    return opts_.minimums_alpha;
  }

  void set_minimums_beta(double v)
  {
    opts_.minimums_beta = v;
  }

  double minimums_beta() const
  {
    return opts_.minimums_beta;
  }

  void set_output_channel(OUTPUT_CHANNEL v)
  {
    output_channel_ = v;
  }

  OUTPUT_CHANNEL output_channel() const
  {
    return output_channel_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  c_locate_extremes_options opts_;
  OUTPUT_CHANNEL output_channel_ = OUTPUT_MASK;
};

#endif /* __c_locate_extremes_routine_h__ */
