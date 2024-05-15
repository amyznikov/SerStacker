/*
 * c_push_image_routine.h
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_push_current_image_routine_h__
#define __c_push_current_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_push_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_push_image_routine,
      "push_image", "c_push_image_routine");

  void set_artifact_name(const std::string & v)
  {
    artifact_name_ = v;
  }

  const std::string & artifact_name() const
  {
    return artifact_name_;
  }

  void set_push_image(bool v)
  {
    push_image_ = v;
  }

  bool push_image() const
  {
    return push_image_;
  }

  void set_push_mask(bool v)
  {
    push_mask_ = v;
  }

  bool push_mask() const
  {
    return push_mask_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  std::string artifact_name_  =
      "saved_image";

  bool push_image_ = true;
  bool push_mask_ = true;

};

#endif /* __c_push_current_image_routine_h__ */
