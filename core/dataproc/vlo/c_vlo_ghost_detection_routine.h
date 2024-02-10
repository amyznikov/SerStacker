/*
 * c_vlo_ghost_detection_routine.h
 *
 *  Created on: Dec 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_ghost_detection_routine_h__
#define __c_vlo_ghost_detection_routine_h__

#include "c_vlo_processor_routine.h"
#include <core/proc/vlo/vlo_ghost_detection.h>

class c_vlo_ghost_detection_routine :
    public c_vlo_processor_routine
{
public:
  DECLARE_VLO_PROCESSOR_CLASS_FACTORY(c_vlo_ghost_detection_routine,
      "vlo_ghost_detection",
      "Detect ghosts on VLO scans");

  void set_saturation_level(double v)
  {
    options_.saturation_level = v;
  }

  double saturation_level() const
  {
    return options_.saturation_level;
  }

  void set_doubled_distanse_systematic_correction(double v)
  {
    options_.doubled_distanse_systematic_correction = v;
  }

  double doubled_distanse_systematic_correction() const
  {
    return options_.doubled_distanse_systematic_correction;
  }

  void set_doubled_distanse_depth_tolerance(double v)
  {
    options_.doubled_distanse_depth_tolerance = v;
  }

  double doubled_distanse_depth_tolerance() const
  {
    return options_.doubled_distanse_depth_tolerance;
  }

  void set_drop_noise_behind_reflector(bool v)
  {
    options_.drop_noise_behind_reflector = v;
  }

  bool drop_noise_behind_reflector() const
  {
    return options_.drop_noise_behind_reflector;
  }

  void set_mask_mode(c_vlo_frame::SELECTION_MASK_MODE v)
  {
    mask_mode_ = v;
  }

  c_vlo_frame::SELECTION_MASK_MODE mask_mode() const
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

  void get_parameters(std::vector<struct c_data_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_vlo_frame * vlo) override;

protected:
  c_vlo_ghost_detection_options options_;
  c_vlo_frame::SELECTION_MASK_MODE mask_mode_ = c_vlo_frame::SELECTION_MASK_REPLACE;
  bool invert_selection_ = true;
};

#endif /* __c_vlo_ghost_detection_routine_h__ */
