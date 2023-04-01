/*
 * QLiveCameraCalibrationPipeline.h
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLiveCameraCalibrationPipeline_h__
#define __QLiveCameraCalibrationPipeline_h__

#include "QLivePipeline.h"
#include <core/pipeline/camera_calibration/c_camera_calibration.h>
#include <core/io/c_output_frame_writer.h>

namespace serimager {

class QLiveCameraCalibrationPipeline :
    public QLivePipeline,
    public c_camera_calibration
{
public:
  typedef QLiveCameraCalibrationPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveCameraCalibrationPipeline(const QString & name, QObject * parent = nullptr);

  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) override;
  bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) override;

  bool serialize(c_config_setting settings, bool save) override;

protected:
  COLORID displayColorid_ = COLORID_UNKNOWN;
  std::string output_path_;
};

} /* namespace serimager */

#endif /* __QLiveCameraCalibrationPipeline_h__ */
