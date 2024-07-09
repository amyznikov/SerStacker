/*
 * c_load_image_routine.h
 *
 *  Created on: May 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_load_image_routine_h__
#define __c_load_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_load_image_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_load_image_routine,
      "load_image", "Load image from file");

  void set_filename(const std::string & v)
  {
    filename_ = v;
  }

  const std::string & filename() const
  {
    return filename_;
  }

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
  std::string filename_;

  std::string artifact_name_  =
      "saved_image";
};

#endif /* __c_load_image_routine_h__ */
