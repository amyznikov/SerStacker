/*
 * QCameraCalibrationInputOptions.cc
 *
 *  Created on: Feb 28, 2023
 *      Author: amyznikov
 */

#include "QCameraCalibrationInputOptions.h"

QCameraCalibrationInputOptions::QCameraCalibrationInputOptions(QWidget * parent) :
    Base("QCameraCalibrationInputOptions", parent)
{
  start_frame_index_ctl =
      add_numeric_box<int>("start_frame_index:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->input_options().start_frame_index = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().start_frame_index;
              return true;
            }
            return false;
          });

  max_input_frames_ctl =
      add_numeric_box<int>("max_input_frames:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->input_options().max_input_frames = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().max_input_frames;
              return true;
            }
            return false;
          });

  inpaint_missing_pixels_ctl =
      add_checkbox("inpaint_missing_pixels",
          [this](bool value) {
            if ( pipeline_ ) {
              pipeline_->input_options().inpaint_missing_pixels = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().inpaint_missing_pixels;
              return true;
            }
            return false;
          });

  enable_color_maxtrix_ctl =
      add_checkbox("enable_color_maxtrix",
          [this](bool value) {
            if ( pipeline_ ) {
              pipeline_->input_options().enable_color_maxtrix = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * value) {
            if ( pipeline_ ) {
              * value = pipeline_->input_options().enable_color_maxtrix;
              return true;
            }
            return false;
          });

}

void QCameraCalibrationInputOptions::set_current_pipeline(const c_camera_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_camera_calibration_pipeline::sptr& QCameraCalibrationInputOptions::current_pipeline() const
{
  return pipeline_;
}

void QCameraCalibrationInputOptions::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    setEnabled(true);
  }
}

