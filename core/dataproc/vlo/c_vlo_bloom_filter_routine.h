/*
 * c_vlo_bloom_filter_routine.h
 *
 *  Created on: Jan 30, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_bloom_filter_routine_h__
#define __c_vlo_bloom_filter_routine_h__

#include "c_vlo_processor_routine.h"
#include <core/proc/vlo/vlo_bloom_detection.h>


class c_vlo_bloom_filter_routine :
    public c_vlo_processor_routine
{
public:
  DECLARE_VLO_PROCESSOR_CLASS_FACTORY(c_vlo_bloom_filter_routine,
      "vlo_bloom_detection",
      "Detect bloom on VLO scans");

  void set_intensity_saturation_level_020m(double v)
  {
    opts_.intensity_saturation_level_020m = v;
  }

  double intensity_saturation_level_020m() const
  {
    return opts_.intensity_saturation_level_020m;
  }

  void set_intensity_saturation_level_100m(double v)
  {
    opts_.intensity_saturation_level_100m = v;
  }

  double intensity_saturation_level_100m() const
  {
    return opts_.intensity_saturation_level_100m;
  }

  void set_bloom_min_intensity(double v)
  {
    opts_.bloom_min_intensity = v;
  }

  double bloom_min_intensity() const
  {
    return opts_.bloom_min_intensity;
  }

  void set_intensity_tolerance(double v)
  {
    opts_.intensity_tolerance = v;
  }

  double intensity_tolerance() const
  {
    return opts_.intensity_tolerance;
  }


  void set_bloom_log_intensity_tolerance(double v)
  {
    opts_.bloom_intensity_tolerance = v;
  }

  double bloom_log_intensity_tolerance() const
  {
    return opts_.bloom_intensity_tolerance;
  }

  void set_max_reflector_hole_size(int v)
  {
    opts_.max_reflector_hole_size = v;
  }

  int max_reflector_hole_size() const
  {
    return opts_.max_reflector_hole_size;
  }

  void set_rreset(bool v)
  {
    opts_.rreset = v;
  }

  bool rreset() const
  {
    return opts_.rreset;
  }

  void set_min_bloom_slope(double v)
  {
    opts_.min_bloom_slope = v;
  }

  double min_bloom_slope() const
  {
    return opts_.min_bloom_slope;
  }

  void set_max_bloom_slope(double v)
  {
    opts_.max_bloom_slope = v;
  }

  double max_bloom_slope() const
  {
    return opts_.max_bloom_slope;
  }


  void set_mask_mode(c_vlo_data_frame::SELECTION_MASK_MODE v)
  {
    mask_mode_ = v;
  }

  c_vlo_data_frame::SELECTION_MASK_MODE mask_mode() const
  {
    return mask_mode_;
  }

  void set_invert_selection(bool v)
  {
    invert_selection_ = v;
  }

  bool invert_selection() const
  {
    return invert_selection_;
  }

//  void set_create_bloom_picture(bool v)
//  {
//    create_bloom_picture_ = true;
//  }
//
//  bool create_bloom_picture() const
//  {
//    return create_bloom_picture_;
//  }

  void set_display_reflectors(bool v)
  {
    display_reflectors_ = v;
  }

  bool display_reflectors() const
  {
    return display_reflectors_;
  }

  void set_display_bloom(bool v)
  {
    display_bloom_ = v;
  }

  bool display_bloom() const
  {
    return display_bloom_;
  }

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

  void set_distance_tolerance(double v)
  {
    opts_.distance_tolerance = v;
  }

  double distance_tolerance() const
  {
    return opts_.distance_tolerance;
  }

  void set_min_segment_height(double v)
  {
    opts_.min_segment_height = v;
  }

  double min_segment_height() const
  {
    return opts_.min_segment_height;
  }

  void set_max_segment_slope(double v)
  {
    opts_.max_segment_slope = v;
  }

  double max_segment_slope() const
  {
    return opts_.max_segment_slope;
  }

  void set_min_segment_size(int v)
  {
    opts_.min_segment_size = v;
  }

  int min_segment_size() const
  {
    return opts_.min_segment_size;
  }

  void set_counts_threshold(int v)
  {
    opts_.counts_threshold = v;
  }

  int counts_threshold() const
  {
    return opts_.counts_threshold;
  }

  void set_intensity_measure(VLO_BLOOM_INTENSITY_MEASURE  v)
  {
    opts_.intensity_measure = v;
  }

  VLO_BLOOM_INTENSITY_MEASURE intensity_measure() const
  {
    return opts_.intensity_measure;
  }

  void get_parameters(std::vector<struct c_data_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_vlo_data_frame * vlo) override;


protected:
  c_vlo_bloom_detection_options opts_;
  c_vlo_data_frame::SELECTION_MASK_MODE mask_mode_ = c_vlo_data_frame::SELECTION_MASK_REPLACE;
  bool invert_selection_ = true;
  // bool create_bloom_picture_ = false;
  bool display_reflectors_ = false;
  bool display_bloom_ = false;
};

#endif /* __c_vlo_bloom_filter_routine_h__ */
