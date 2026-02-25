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

class c_star_extractor_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_star_extractor_routine,
      "star_extractor", "Extract stars on sky image");

  enum DisplayType {
    DisplayRichKeypoints,
    DisplaySourceImage,
    DisplayFilteredImage,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DisplayType _display_type = DisplayRichKeypoints;
  int _median_filter_size = 3;
  double _sigma1 = 10;
  double _sigma2 = 2;
  double _noise_sigma = 100;
  double _noise_scale = 10;
};

#endif /* __c_star_extractor_routine_h__ */
