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
    public QLivePipeline

{
public:
  typedef QLiveCameraCalibrationPipeline ThisClass;
  typedef QLivePipeline Base;

  QLiveCameraCalibrationPipeline(const QString & name, QObject * parent = nullptr);

  c_camera_calibration & camera_calibration();
  const c_camera_calibration & camera_calibration() const;

  void set_save_frames_with_detected_chessboard(bool v);
  bool save_frames_with_detected_chessboard() const;

  void set_frames_with_detected_chessboard_filename(const QString & v);
  const QString & frames_with_detected_chessboard_filename() const;

  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) override;
  bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) override;

  bool serialize(c_config_setting settings, bool save) override;

protected:
  COLORID displayColorid_ = COLORID_UNKNOWN;
  c_camera_calibration camera_calibration_;
  std::string output_path_;

  bool save_frames_with_detected_chessboard_ = false;
  QString frames_with_detected_chessboard_filename_;

  c_output_frame_writer frame_writer_;

};

} /* namespace serimager */

#endif /* __QLiveCameraCalibrationPipeline_h__ */
