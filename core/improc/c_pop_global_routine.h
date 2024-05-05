/*
 * c_pop_global_routine.h
 *
 *  Created on: May 5, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pop_global_routine_h__
#define __c_pop_global_routine_h__

#include <core/improc/c_image_processor.h>

class c_pop_global_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_pop_global_routine,
      "pop_global", "c_pop_global_routine");

  void set_artifact_name(const std::string & v)
  {
    artifact_name_ = v;
  }

  const std::string & artifact_name() const
  {
    return artifact_name_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  std::string artifact_name_  =
      "saved_image";

};

#endif /* __c_pop_global_routine_h__ */
