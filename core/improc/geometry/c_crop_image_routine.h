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
#include <core/proc/feature2d/c_roi_selection.h>
#include <core/proc/feature2d/c_planetary_disk_selection.h>
#include <core/proc/feature2d/planetary-disk-detection.h>

class c_crop_image_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_crop_image_routine,
      "crop", "Crop rectangle region of image");

  enum SELECTION_MODE {
    ROI_SELECTION_NONE,
    ROI_SELECTION_FIXED_RECT,
    ROI_SELECTION_GUI,
    ROI_SELECTION_PLANETARY_DISK,
    // ROI_SELECTION_MAX_CONNECTED_COMPONENT,
  };

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  SELECTION_MODE mode = ROI_SELECTION_GUI;
  cv::BorderTypes borderType = cv::BORDER_DEFAULT;
  cv::Scalar borderValue;
  cv::Size outputSize;
  bool fixOutputSize = false;

  struct {
    cv::Rect rc;
  } manualRectOptions;

  struct {
    c_simple_planetary_disk_detector_options opts;
    bool putToCenter = false;
  } planetaryDiskOptions;

};

#endif /* __c_crop_image_routine_h__ */
