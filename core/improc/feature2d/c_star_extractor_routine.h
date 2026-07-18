/*
 * c_star_extractor_routine.h
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_star_extractor_routine_h__
#define __c_star_extractor_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/feature2d/c_simple_star_detector.h>


class c_star_extractor_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_star_extractor_routine,
      "star_extractor", "Extract stars on sky image");

  enum DisplayType {
    DisplaySourceImage,
    DisplayDogImage,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DisplayType _display_type = DisplayDogImage;
  bool _display_blobs = true;
  c_simple_star_detector_options _opts;
  c_simple_star_detector _detector;
};

#endif /* __c_star_extractor_routine_h__ */
