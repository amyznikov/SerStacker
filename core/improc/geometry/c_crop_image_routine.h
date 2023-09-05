/*
 * c_crop_image_routine.h
 *
 *  Created on: Aug 31, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_crop_image_routine_h__
#define __c_crop_image_routine_h__

#include <core/improc/c_image_processor.h>

class c_crop_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_crop_image_routine,
      "crop", "Crop rectangle region of image");

  void set_rect(const cv::Rect & rc);
  const cv::Rect & rect() const;

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::Rect rect_;
};

#endif /* __c_crop_image_routine_h__ */
