/*
 * c_vlo_echo_mask_routine.cc
 *
 *  Created on: Dec 17, 2023
 *      Author: amyznikov
 */

#include "c_vlo_echo_mask_routine.h"

void c_vlo_echo_mask_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_CTRL(ctls, enable_echo0, "echo0",  "Enable Echo0");
  BIND_CTRL(ctls, enable_echo1, "echo1",  "Enable Echo0");
  BIND_CTRL(ctls, enable_echo2, "echo2",  "Enable Echo0");
  BIND_CTRL(ctls, invert_selection, "invert_selection", "invert_selection");
  BIND_CTRL(ctls, mask_mode, "mask_mode", "combine selection mode");
}

bool c_vlo_echo_mask_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, echo_mask);
    SERIALIZE_PROPERTY(settings, save, *this, invert_selection);
    SERIALIZE_PROPERTY(settings, save, *this, mask_mode);
    return true;
  }

  return false;
}

bool c_vlo_echo_mask_routine::process(c_vlo_data_frame * vlo)
{
  const uint8_t e0 = ((echo_mask_ & c_vlo_data_frame::ECHO0));
  const uint8_t e1 = ((echo_mask_ & c_vlo_data_frame::ECHO1));
  const uint8_t e2 = ((echo_mask_ & c_vlo_data_frame::ECHO2));

  const cv::Mat3b echo_mask(vlo->current_scan_.size,
      invert_selection_ ? cv::Vec3b(255 * (e0 == 0), 255 * (e1 == 0), 255 * (e2 == 0)) :
          cv::Vec3b(255 * (e0 != 0), 255 * (e1 != 0), 255 * (e2 != 0)));

  vlo->update_selection(echo_mask, mask_mode_);

  return true;
}
