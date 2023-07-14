/*
 * QStereoCalibrationInputOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QStereoInputOptions.h"
//
//QStereoInputOptions::QStereoInputOptions(QWidget * parent) :
//    Base("QStereoCalibrationInputOptions", parent)
//{
//
//
//  start_frame_index_ctl =
//      add_numeric_box<int>("start_frame_index:",
//          "",
//          [this](int value) {
//            if ( options_ ) {
//              options_->start_frame_index = value;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * value) {
//            if ( options_ ) {
//              * value = options_->start_frame_index;
//              return true;
//            }
//            return false;
//          });
//
//  max_input_frames_ctl =
//      add_numeric_box<int>("max_input_frames:",
//          "",
//          [this](int value) {
//            if ( options_ ) {
//              options_->max_input_frames = value;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](int * value) {
//            if ( options_ ) {
//              * value = options_->max_input_frames;
//              return true;
//            }
//            return false;
//          });
//
//  inpaint_missing_pixels_ctl =
//      add_checkbox("inpaint_missing_pixels",
//          "",
//          [this](bool value) {
//            if ( options_ ) {
//              options_->inpaint_missing_pixels = value;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](bool * value) {
//            if ( options_ ) {
//              * value = options_->inpaint_missing_pixels;
//              return true;
//            }
//            return false;
//          });
//
//  enable_color_maxtrix_ctl =
//      add_checkbox("enable_color_maxtrix",
//          "",
//          [this](bool value) {
//            if ( options_ ) {
//              options_->enable_color_maxtrix = value;
//              Q_EMIT parameterChanged();
//            }
//          },
//          [this](bool * value) {
//            if ( options_ ) {
//              * value = options_->enable_color_maxtrix;
//              return true;
//            }
//            return false;
//          });
//
//  updateControls();
//}
//
//void QStereoInputOptions::set_input_options(c_stereo_input_options * options)
//{
//  options_ = options;
//  updateControls();
//}
//
//c_stereo_input_options * QStereoInputOptions::input_options() const
//{
//  return options_;
//}
//
//void QStereoInputOptions::populatesources()
//{
//}
//
//void QStereoInputOptions::updatesourcecontrols()
//{
//
//}
//
//void QStereoInputOptions::onupdatecontrols()
//{
//  populatesources();
//
//  if( !options_ || !options_->input_sequence ) {
//    setEnabled(false);
//  }
//  else {
//
//    Base::onupdatecontrols();
//    updatesourcecontrols();
//
//    setEnabled(true);
//  }
//}

