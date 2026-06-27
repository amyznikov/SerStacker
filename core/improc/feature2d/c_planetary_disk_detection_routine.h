/*
 * c_planetary_disk_detection_routine.h
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_planetary_disk_detection_routine_h__
#define __c_planetary_disk_detection_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/extract_channel.h>
#include <core/proc/pixtype.h>

class c_planetary_disk_detection_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_planetary_disk_detection_routine,
      "planetary_disk_detection", "Calls simple_planetary_disk_detector()");

  enum DISPLAY {
    DISPLAY_SRC_IMAGE,
    DISPLAY_INTENSITY_IMAGE,
    DISPLAY_SPECTRUM_MODULE
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  DISPLAY _display = DISPLAY_SRC_IMAGE;
  double gsigma = 1;
  int se_radius = 5;
  enum color_channel_type _intensity_channel = color_channel_gray;
  bool updateROI = false;

  // work arrays
  cv::Mat INTENSITY_IMAGE;
  cv::Mat2f INTENSITY_P, INTENSITY_S;
  cv::Mat1f INTENSITY_Magnitude;
  cv::Mat1f VLAP;
  cv::Mat1f ApodizationWindow;
};

#endif /* __c_planetary_disk_detection_routine_h__ */
