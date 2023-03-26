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

namespace serimager {

class QLiveCameraCalibrationPipeline :
    public QLivePipeline

{
public:
  typedef QLiveCameraCalibrationPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveCameraCalibrationPipeline(const QString & name, QObject * parent = nullptr);

  c_camera_calibration & camera_calibration();
  const c_camera_calibration & camera_calibration() const;

  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) override;
  bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) override;

  bool serialize(c_config_setting settings, bool save) override;

protected:
  COLORID displayColorid_ = COLORID_UNKNOWN;
  c_camera_calibration camera_calibration_;
  QString output_path_;

};

} /* namespace serimager */

#endif /* __QLiveCameraCalibrationPipeline_h__ */
