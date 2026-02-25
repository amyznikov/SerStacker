/*
 * c_local_minmax_routine.h
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_local_minmax_routine_h__
#define __c_local_minmax_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/locate_extremes.h>


class c_local_minmax_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_local_minmax_routine,
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

  void set_filter_type(c_locate_extremes_options::neighbor_filter_type v)
  {
    _opts.filter_type = v;
  }

  c_locate_extremes_options::neighbor_filter_type filter_type() const
  {
    return _opts.filter_type;
  }

  void set_se_shape(cv::MorphShapes v)
  {
    _opts.se_shape = v;
  }

  cv::MorphShapes se_shape() const
  {
    return _opts.se_shape;
  }

  void set_se_size(const cv::Size & v)
  {
    _opts.se_size = v;
  }

  const cv::Size& se_size() const
  {
    return _opts.se_size;
  }

  void set_anchor(const cv::Point & v)
  {
    _opts.anchor = v;
  }

  const cv::Point & anchor() const
  {
    return _opts.anchor;
  }

  void set_border_type(BorderType v)
  {
    _opts.border_type = static_cast<cv::BorderTypes>(v);
  }

  BorderType border_type() const
  {
    return static_cast<BorderType>(_opts.border_type);
  }

  void set_border_value(const cv::Scalar & v)
  {
    _opts.border_value = v;
  }

  const cv::Scalar & border_value() const
  {
    return _opts.border_value;
  }

  void set_locate_maximums(bool v)
  {
    _opts.locate_maximums = v;
  }

  bool locate_maximums() const
  {
    return _opts.locate_maximums;
  }

  void set_locate_minimums(bool v)
  {
    _opts.locate_minimums = v;
  }

  bool locate_minimums() const
  {
    return _opts.locate_minimums;
  }

  void set_maximums_alpha(double v)
  {
    _opts.maximums_alpha = v;
  }

  double maximums_alpha() const
  {
    return _opts.maximums_alpha;
  }

  void set_maximums_beta(double v)
  {
    _opts.maximums_beta = v;
  }

  double maximums_beta() const
  {
    return _opts.maximums_beta;
  }

  void set_minimums_alpha(double v)
  {
    _opts.minimums_alpha = v;
  }

  double minimums_alpha() const
  {
    return _opts.minimums_alpha;
  }

  void set_minimums_beta(double v)
  {
    _opts.minimums_beta = v;
  }

  double minimums_beta() const
  {
    return _opts.minimums_beta;
  }

  void set_output_channel(OUTPUT_CHANNEL v)
  {
    _output_channel = v;
  }

  OUTPUT_CHANNEL output_channel() const
  {
    return _output_channel;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_locate_extremes_options _opts;
  OUTPUT_CHANNEL _output_channel = OUTPUT_MASK;
};

#endif /* __c_local_minmax_routine_h__ */
