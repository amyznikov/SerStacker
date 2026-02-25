/*
 * c_find_chessboard_corners_routine.h
 *
 *  Created on: Feb 25, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_find_chessboard_corners_routine_h__
#define __c_find_chessboard_corners_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/chessboard/chessboard_detection.h>

class c_find_chessboard_corners_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_find_chessboard_corners_routine,
      "find_chessboard_corners",
      "Use cv::findChessboardCorners() to find chessboard corners on image");

  enum DisplayType {
    DisplayCorners = 0,
    DisplayOtsuImage,
    DisplayHarrisImage,
    DisplayHarrisThresholdImage,
    DisplayHHarrisImage,
    DisplayVHarrisImage
  };


  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_chessboard_corners_detection_options _options;
  enum DisplayType _display_type = DisplayCorners;
  bool _stereo = false;
};

#endif /* __c_find_chessboard_corners_routine_h__ */
