/*
 * c_image_gradient_routine.h
 *
 *  Created on: Jan 11, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_gradient_routine_h__
#define __c_image_gradient_routine_h__

#include "c_video_frame_processor_routine.h"

class c_image_gradient_routine :
    public c_video_frame_processor_routine
{
public:
  DECLARE_VIDEO_FRAME_PROCESSOR_CLASS_FACTORY(c_image_gradient_routine,
      "gradient",
      "compute image gradient");

  const std::string & input_image_name() const;
  void set_input_image_name(const std::string & v);

  const std::string & output_image_name() const;
  void set_output_image_name(const std::string & v);

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(c_video_frame * vlo) override;

protected:
  std::string input_image_name_;
  std::string output_image_name_;
};

#endif /* __c_image_gradient_routine_h__ */
