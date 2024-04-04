/*
 * c_vlo_ghost_detection_routine.cc
 *
 *  Created on: Dec 19, 2023
 *      Author: amyznikov
 */

#include "c_vlo_ghost_detection_routine.h"
#include <core/debug.h>

void c_vlo_ghost_detection_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_CTRL(ctls, saturation_level, "saturation_level", "saturation_level");
  BIND_CTRL(ctls, doubled_distanse_depth_tolerance, "depth_tolerance", "doubled_distanse_depth_tolerance");
  BIND_CTRL(ctls, doubled_distanse_systematic_correction, "systematic_correction", "doubled_distanse_systematic_correction");
  BIND_CTRL(ctls, drop_noise_behind_reflector, "drop_noise_behind_reflector", "drop_noise_behind_reflector");
  BIND_CTRL(ctls, invert_selection, "invert_selection", "invert_selection");
  BIND_CTRL(ctls, mask_mode, "mask_mode", "combine selection mode");

}

bool c_vlo_ghost_detection_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, saturation_level);
    SERIALIZE_PROPERTY(settings, save, *this, doubled_distanse_depth_tolerance);
    SERIALIZE_PROPERTY(settings, save, *this, doubled_distanse_systematic_correction);
    SERIALIZE_PROPERTY(settings, save, *this, drop_noise_behind_reflector);
    SERIALIZE_PROPERTY(settings, save, *this, invert_selection);
    SERIALIZE_PROPERTY(settings, save, *this, mask_mode);
    return true;
  }
  return false;
}

bool c_vlo_ghost_detection_routine::process(c_vlo_data_frame * vlo)
{
  cv::Mat ghost_mask;

  if( !vlo_ghost_detection(vlo->current_scan_, options_, ghost_mask) || ghost_mask.empty() ) {
    CF_ERROR("vlo_ghost_detection() fails");
    return false;
  }

  if ( invert_selection_ ) {
    cv::bitwise_not(ghost_mask, ghost_mask);
  }

  vlo->update_selection(ghost_mask,
      mask_mode_);

  return true;
}

