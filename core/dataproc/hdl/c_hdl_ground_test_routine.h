/*
 * c_hdl_ground_test_routine.h
 *
 *  Created on: Mar 22, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_ground_test_routine_h__
#define __c_hdl_ground_test_routine_h__

#include "c_hdl_processor_routine.h"

class c_hdl_ground_test_routine :
    public c_hdl_processor_routine
{
public:
  DECLARE_HDL_PROCESSOR_CLASS_FACTORY(c_hdl_ground_test_routine,
      "hdl_ground_test",
      "Test for round detection on HDL range images");

  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);
  bool serialize(c_config_setting settings, bool save) final;
  bool process(c_hdl_data_frame * hdl) final;

protected:
  float H0 = 2; // [m]
  float Zroof = -1; // [m]
  float gtol = 0.1; // [m]
  bool updateSelectionMask = false;

};

#endif /* __c_hdl_ground_test_routine_h__ */
