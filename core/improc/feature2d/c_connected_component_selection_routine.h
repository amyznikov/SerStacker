/*
 * c_connected_component_selection_routine.h
 *
 *  Created on: Jul 13, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_connected_component_selection_routine_h_
#define __c_connected_component_selection_routine_h_

#include <core/improc/c_image_processor.h>

class c_connected_component_selection_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_connected_component_selection_routine,
      "connected_component_selection", "Select singke connected component from pixel mask");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  bool initialize() final
  {
    _input_channel = MASK;
    _output_channel = MASK;
    return true;
  }

protected:
  COMBINE_MASK_MODE _mask_mode = COMBINE_MASK_MODE_REPLACE;
  bool _update_roi = false;
  bool _extend_roi_to_optimal_fft_size = false;
};

#endif /* __c_connected_component_selection_routine_h_ */
