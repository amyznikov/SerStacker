/*
 * QCameraCalibrationOutputOptions.h
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraCalibrationOutputOptions_h__
#define __QCameraCalibrationOutputOptions_h__

#include <gui/qpipeline/QPipelineOutputOptions.h>
#include <core/pipeline/c_camera_calibration_pipeline/c_camera_calibration_pipeline.h>

class QCameraCalibrationOutputOptions :
    public QPipelineOutputOptions<c_camera_calibration_output_options>
{
public:
  typedef QCameraCalibrationOutputOptions ThisClass;
  typedef QPipelineOutputOptions<c_camera_calibration_output_options> Base;

  QCameraCalibrationOutputOptions(QWidget * parent = nullptr);

protected:
  void onupdatecontrols() override;

protected:
  QCheckBox * save_chessboard_frames_ctl = nullptr;
  QLineEditBox * chessboard_frames_filename_ctl = nullptr;

  QCheckBox * save_rectified_frames_ctl = nullptr;
  QLineEditBox * rectified_frames_filename_ctl = nullptr;

  QCheckBox * save_progress_video_ctl = nullptr;
  QLineEditBox * progress_video_filename_ctl = nullptr;

};


#endif /* __QCameraCalibrationOutputOptions_h__ */
