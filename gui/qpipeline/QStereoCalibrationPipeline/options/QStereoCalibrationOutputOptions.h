/*
 * QStereoCalibrationOutputOptions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStereoCalibrationOutputOptions_h__
#define __QStereoCalibrationOutputOptions_h__


#include <gui/qpipeline/QPipelineOutputOptions.h>
#include <core/pipeline/c_stereo_calibration_pipeline/c_stereo_calibration_pipeline.h>

class QStereoCalibrationOutputOptions :
    public QPipelineOutputOptions<c_stereo_calibration_output_options>
{
public:
  typedef QStereoCalibrationOutputOptions ThisClass;
  typedef QPipelineOutputOptions<c_stereo_calibration_output_options> Base;

  QStereoCalibrationOutputOptions(QWidget * parent = nullptr);


protected:
  void onupdatecontrols() override;

protected:
  QCheckBox * save_chessboard_frames_ctl = nullptr;
  QLineEditBox * chessboard_frames_filename_ctl = nullptr;

  QCheckBox * save_calibration_progress_video_ctl = nullptr;
  QLineEditBox * calibration_progress_filename_ctl = nullptr;

  QCheckBox * save_rectified_images_ctl = nullptr;
  QLineEditBox * rectified_images_filename_ctl = nullptr;

  QCheckBox * save_stereo_rectified_frames_ctl = nullptr;
  QLineEditBox * stereo_rectified_frames_filename_ctl = nullptr;

  QCheckBox * save_quad_rectified_frames_ctl = nullptr;
  QLineEditBox * quad_rectified_frames_filename_ctl = nullptr;


};

#endif /* __QStereoCalibrationOutputOptions_h__ */
