/*
 * QRStereoCalibrateOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QRStereoCalibrateOptions.h"

QRStereoCalibrateOptions::QRStereoCalibrateOptions(QWidget * parent) :
    Base("QRStereoCalibrateOptions", parent)
{
  min_frames_ctl =
      add_numeric_box<int>("min_frames:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->stereo_calibrate_options().min_frames = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->stereo_calibrate_options().min_frames;
              return true;
            }
            return false;
          });

  max_frames_ctl =
      add_numeric_box<int>("max_frames:",
          [this](int value) {
            if ( pipeline_ ) {
              pipeline_->stereo_calibrate_options().max_frames = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( pipeline_ ) {
              * value = pipeline_->stereo_calibrate_options().max_frames;
              return true;
            }
            return false;
          });
//
//  max_iterations_ctl =
//      add_numeric_box<int>("max_iterations:",
//          [this](int value) {
//            if ( pipeline_ ) {
//              cv::TermCriteria & t =
//                  pipeline_->stereo_calibrate_options().solverTerm;
//              if ( (t.maxCount = value) > 0 ) {
//                t.type |= cv::TermCriteria::COUNT;
//              }
//              else {
//                t.type &= ~cv::TermCriteria::COUNT;
//              }
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * value) {
//            if ( pipeline_ ) {
//              * value = pipeline_->stereo_calibrate_options().solverTerm.maxCount;
//              return true;
//            }
//            return false;
//          });
//
//  eps_ctl =
//      add_numeric_box<double>("eps:",
//          [this](double value) {
//            if ( pipeline_ ) {
//              cv::TermCriteria & t =
//                  pipeline_->stereo_calibrate_options().solverTerm;
//              if ( (t.epsilon = value) >= 0 ) {
//                t.type |= cv::TermCriteria::EPS;
//              }
//              else {
//                t.type &= ~cv::TermCriteria::EPS;
//              }
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](double * value) {
//            if ( pipeline_ ) {
//              * value = pipeline_->stereo_calibrate_options().solverTerm.epsilon;
//              return true;
//            }
//            return false;
//          });
//
//  calibration_flags_ctl =
//      add_flags_editbox<STEREO_CALIBRATION_FLAGS>("calibration_flags:",
//          [this](int value) {
//            if ( pipeline_ ) {
//              pipeline_->stereo_calibrate_options().calibration_flags = value;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * value) {
//            if ( pipeline_ ) {
//              * value = pipeline_->stereo_calibrate_options().calibration_flags;
//              return true;
//            }
//            return false;
//          });
//
//  auto_tune_calibration_flags_ctl =
//      add_checkbox("auto_tune_calibration_flags",
//          [this](bool checked) {
//            if ( pipeline_ ) {
//              pipeline_->stereo_calibrate_options().auto_tune_calibration_flags = checked;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](bool * checked ) {
//            if ( pipeline_ ) {
//              * checked = pipeline_->stereo_calibrate_options().auto_tune_calibration_flags;
//              return true;
//            }
//            return false;
//          });
//
  filter_alpha_ctl =
      add_numeric_box<double>("filter_alpha:",
          [this](double value) {
            if ( pipeline_ ) {
              pipeline_->stereo_calibrate_options().filter_alpha = value;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * value) {
            if ( pipeline_ ) {
              * value = pipeline_->stereo_calibrate_options().filter_alpha;
              return true;
            }
            return false;
          });


  updateControls();
}

void QRStereoCalibrateOptions::set_current_pipeline(const c_rstereo_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_rstereo_calibration_pipeline::sptr& QRStereoCalibrateOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoCalibrateOptions::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    setEnabled(true);
  }
}
