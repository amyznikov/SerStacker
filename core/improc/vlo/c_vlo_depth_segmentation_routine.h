/*
 * c_vlo_depth_segmentation_routine.h
 *
 *  Created on: Nov 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_depth_segmentation_routine_h__
#define __c_vlo_depth_segmentation_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/vlo/vlo.h>


class c_vlo_depth_segmentation_routine :
    public c_image_processor_routine
{
public:

  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_vlo_depth_segmentation_routine,
      "vlo_depth_segmentation", "calls vlo_depth_segmentation())");

  void set_min_distance(double v)
  {
    opts_.min_distance = v;
  }

  double min_distance() const
  {
    return opts_.min_distance;
  }

  void set_max_distance(double v)
  {
    opts_.max_distance = v;
  }

  double max_distance() const
  {
    return opts_.max_distance;
  }

  void set_vlo_walk_error(double v)
  {
    opts_.vlo_walk_error = v;
  }

  double vlo_walk_error() const
  {
    return opts_.vlo_walk_error;
  }

  void set_max_slope(double v)
  {
    opts_.max_slope = v;
  }

  double max_slope() const
  {
    return opts_.max_slope;
  }

  void set_min_slope(double v)
  {
    opts_.min_slope = v;
  }

  double min_slope() const
  {
    return opts_.min_slope;
  }

  void set_min_pts(int v)
  {
    opts_.min_pts = v;
  }

  int min_pts() const
  {
    return opts_.min_pts;
  }

  void set_output_type(vlo_depth_segmentation_output_type v)
  {
    opts_.output_type = v;
  }

  vlo_depth_segmentation_output_type output_type() const
  {
    return opts_.output_type;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save);
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override;

protected:
  c_vlo_depth_segmentation_options opts_;
};

#endif /* __c_vlo_depth_segmentation_routine_h__ */
