/*
 * QCalibrateCameraOptions.cc
 *
 *  Created on: Feb 28, 2023
 *      Author: amyznikov
 */

#include "QCalibrateCameraOptions.h"

QCalibrateCameraOptions::QCalibrateCameraOptions(QWidget * parent) :
    Base("QCalibrateCameraOptions", parent)
{
  enable_calibration_ctl =
      add_checkbox("Enable Calibration",
          "Enable calls to calibrate_camera().\n"
              "When this option is disabled no actual calibration will done"
              "but using 'Output options' you can just save frames on which landmarks are detected\n",
          [this](bool checked) {
            if ( options_ && options_->enable_calibration != checked ) {
              options_->enable_calibration = checked;
              updatecontrolstate();
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->enable_calibration;
              return true;
            }
            return false;
          });

  min_frames_ctl =
      add_numeric_box<int>("min_frames:",
          "Minimal number of accumulated frames with detected chessboard corners to start camera calibration ",
          [this](int value) {
            if ( options_ ) {
              options_->min_frames = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->min_frames;
              return true;
            }
            return false;
          });

  max_frames_ctl =
      add_numeric_box<int>("max_frames:",
          "Maximal number of accumulated frames for camera calibration. "
          "The pipeline will select most best 'max_frames' from whole dataset for calibration.",
          [this](int value) {
            if ( options_ ) {
              options_->max_frames = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->max_frames;
              return true;
            }
            return false;
          });

  max_iterations_ctl =
      add_numeric_box<int>("max_iterations:",
          "Max iteration for internal solver",
          [this](int value) {
            if ( options_ ) {
              cv::TermCriteria & t =
                  options_->solverTerm;
              if ( (t.maxCount = value) > 0 ) {
                t.type |= cv::TermCriteria::COUNT;
              }
              else {
                t.type &= ~cv::TermCriteria::COUNT;
              }
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->solverTerm.maxCount;
              return true;
            }
            return false;
          });

  eps_ctl =
      add_numeric_box<double>("eps:",
          "Term criteria for internal solver",
          [this](double value) {
            if ( options_ ) {
              cv::TermCriteria & t =
                  options_->solverTerm;
              if ( (t.epsilon = value) >= 0 ) {
                t.type |= cv::TermCriteria::EPS;
              }
              else {
                t.type &= ~cv::TermCriteria::EPS;
              }
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( options_ ) {
              * value = options_->solverTerm.epsilon;
              return true;
            }
            return false;
          });

  init_camera_matrix_2d_ctl =
      add_checkbox("initCameraMatrix2D:",
          "Call cv::initCameraMatrix2D() to initialize camera matrix on start",
          [this](bool checked) {
            if ( options_ ) {
              options_->init_camera_matrix_2d = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->init_camera_matrix_2d;
              return true;
            }
            return false;
          });

  calibration_flags_ctl =
      add_flags_editbox<CAMERA_CALIBRATION_FLAGS>("calibration_flags:",
          "Flags for cv::calibrateCamera()",
          [this](int value) {
            if ( options_ ) {
              options_->calibration_flags = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( options_ ) {
              * value = options_->calibration_flags;
              return true;
            }
            return false;
          });

  auto_tune_calibration_flags_ctl =
      add_checkbox("auto_tune_calibration_flags",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->auto_tune_calibration_flags = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked ) {
            if ( options_ ) {
              * checked = options_->auto_tune_calibration_flags;
              return true;
            }
            return false;
          });

  filter_alpha_ctl =
      add_numeric_box<double>("filter_alpha:",
          "Parametr to the quality of current subset :\n"
              "quality = rmse_quality * alpha + coverage_quality * (1-alpha)",
          [this](double value) {
            if ( options_ ) {
              options_->filter_alpha = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( options_ ) {
              * value = options_->filter_alpha;
              return true;
            }
            return false;
          });


  updateControls();
}

void QCalibrateCameraOptions::set_options(c_calibrate_camera_options * options)
{
  options_ = options;
  updateControls();
}

c_calibrate_camera_options * QCalibrateCameraOptions::options() const
{
  return options_;
}


void QCalibrateCameraOptions::updatecontrolstate()
{
  if( options_ ) {

    const bool enable_calibration =
        options_->enable_calibration;

    min_frames_ctl->setEnabled(enable_calibration);
    max_frames_ctl->setEnabled(enable_calibration);
    max_iterations_ctl->setEnabled(enable_calibration);
    eps_ctl->setEnabled(enable_calibration);
    calibration_flags_ctl->setEnabled(enable_calibration);
    init_camera_matrix_2d_ctl->setEnabled(enable_calibration);
    auto_tune_calibration_flags_ctl->setEnabled(enable_calibration);
    filter_alpha_ctl->setEnabled(enable_calibration);
  }
}

void QCalibrateCameraOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    setEnabled(true);
  }
}
