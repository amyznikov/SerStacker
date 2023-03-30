/*
 * QStereoCalibrateOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrateOptions.h"

QStereoCalibrateOptions::QStereoCalibrateOptions(QWidget * parent) :
    Base("QStereoCalibrateOptions", parent)
{
  min_frames_ctl =
      add_numeric_box<int>("min_frames:",
          "Minimal frames accumulated to start stereo_calibrate()",
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
          "Maximal number of best frames to use with stereo_calibrate()",
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
          "Term criteria for cv::stereoCalibrate()",
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
          "Term criteria for cv::stereoCalibrate()",
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
      add_flags_editbox<STEREO_CALIBRATION_FLAGS>("calibration_flags:",
          "Calibration flags for cv::stereoCalibrate()",
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
      add_checkbox("auto_tune_flags:",
          "Auto adjust some of calibration flags for cv::stereoCalibrate()",
          [this](bool checked) {
            if ( options_ ) {
              options_->auto_tune_calibration_flags = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              * checked = options_->auto_tune_calibration_flags;
              return true;
            }
            return false;
          });

  filter_alpha_ctl =
      add_numeric_box<double>("filter_alpha:",
          "Parameter for subset quality estimation:\n"
              " SubsetQuality = RMSE_Quality * alpha + Coverage_Quality * (1-alpha).\n"
              " Ignored in live mode.",
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

void QStereoCalibrateOptions::set_options(c_stereo_calibrate_options * options)
{
  options_ = options;
  updateControls();
}

c_stereo_calibrate_options* QStereoCalibrateOptions::options() const
{
  return options_;
}

void QStereoCalibrateOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    setEnabled(true);
  }
}
