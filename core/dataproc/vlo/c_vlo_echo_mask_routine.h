/*
 * c_vlo_echo_mask_routine.h
 *
 *  Created on: Dec 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_echo_mask_routine_h__
#define __c_vlo_echo_mask_routine_h__

#include "c_vlo_processor_routine.h"


class c_vlo_echo_mask_routine :
    public c_vlo_processor_routine
{
public:
  DECLARE_VLO_PROCESSOR_CLASS_FACTORY(c_vlo_echo_mask_routine,
      "vlo_echo_mask",
      "Select specific echos");

  void set_echo_mask(int v)
  {
    echo_mask_ = v;
  }

  int echo_mask() const
  {
    return echo_mask_;;
  }

  void set_enable_echo0(bool v)
  {
    if ( v ) {
      echo_mask_ |= c_vlo_data_frame::ECHO0;
    }
    else {
      echo_mask_ &= ~c_vlo_data_frame::ECHO0;
    }
  }

  bool enable_echo0() const
  {
    return echo_mask_ & c_vlo_data_frame::ECHO0;
  }

  void set_enable_echo1(bool v)
  {
    if ( v ) {
      echo_mask_ |= c_vlo_data_frame::ECHO1;
    }
    else {
      echo_mask_ &= ~c_vlo_data_frame::ECHO1;
    }
  }

  bool enable_echo1() const
  {
    return echo_mask_ & c_vlo_data_frame::ECHO1;
  }

  void set_enable_echo2(bool v)
  {
    if ( v ) {
      echo_mask_ |= c_vlo_data_frame::ECHO2;
    }
    else {
      echo_mask_ &= ~c_vlo_data_frame::ECHO2;
    }
  }

  bool enable_echo2() const
  {
    return echo_mask_ & c_vlo_data_frame::ECHO2;
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

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_vlo_data_frame * vlo) override;

protected:
  int echo_mask_ = 0xFF;
  c_vlo_data_frame::SELECTION_MASK_MODE mask_mode_ = c_vlo_data_frame::SELECTION_MASK_AND;
  bool invert_selection_ = false;
};

#endif /* __c_vlo_echo_mask_routine_h__ */
