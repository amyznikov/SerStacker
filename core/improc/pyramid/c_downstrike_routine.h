/*
 * c_downstrike_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_downstrike_routine_h__
#define __c_downstrike_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/downstrike.h>

class c_downstrike_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_downstrike_routine,
      "downstrike",
      "2x downsampling step by rejecting each even (uneven) row and column, keep only uneven (even)");

  enum DownstrikeMode {
    DownstrikeUneven,
    DownstrikeEven,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DownstrikeMode _mode = DownstrikeUneven;
};

#endif /* __c_downstrike_routine_h__ */
