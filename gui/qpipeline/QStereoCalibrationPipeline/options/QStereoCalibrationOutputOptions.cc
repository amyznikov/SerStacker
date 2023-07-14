/*
 * QStereoCalibrationOutputOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrationOutputOptions.h"


QStereoCalibrationOutputOptions::QStereoCalibrationOutputOptions(QWidget * parent) :
  Base(parent)
{
  ///

  save_chessboard_frames_ctl =
      add_checkbox("Save chessboard frames:",
          "Save frames with chessboard detected",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_chessboard_frames = checked;
              chessboard_frames_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_chessboard_frames;
              return true;
            }
            return false;
          });

  chessboard_frames_filename_ctl =
      add_textbox("Chessboard video filename:",
          "Output file name for frames with chessboard detected",
          [this](const QString & value) {
            if ( options_ ) {
              options_->chessboard_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->chessboard_frames_filename.c_str();
              return true;
            }
            return false;
          });

  chessboard_frames_filename_ctl->setPlaceholderText("auto");

  ///

  save_calibration_progress_video_ctl =
      add_checkbox("Save progress video:",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_progress_video = checked;
              calibration_progress_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_progress_video;
              return true;
            }
            return false;
          });

  calibration_progress_filename_ctl =
      add_textbox("Progress video filename:",
          "",
          [this](const QString & value) {
            if ( options_ ) {
              options_->progress_video_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->progress_video_filename.c_str();
              return true;
            }
            return false;
          });

  calibration_progress_filename_ctl->setPlaceholderText("auto");

  ///

  save_rectified_images_ctl =
      add_checkbox("Save rectified frames",
          "Save rectified frames in two separate left and right videos",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_rectified_frames = checked;
              rectified_images_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_rectified_frames;
              return true;
            }
            return false;
          });

  rectified_images_filename_ctl =
      add_textbox("Rectified frames filename:",
          "Save rectified frames in two separate left and right videos",
          [this](const QString & value) {
            if ( options_ ) {
              options_->rectified_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->rectified_frames_filename.c_str();
              return true;
            }
            return false;
          });

  rectified_images_filename_ctl->setPlaceholderText("auto");

  ///

  save_stereo_rectified_frames_ctl =
      add_checkbox("Save stereo rectified frames",
          "Save rectified frames laid out horizontally into single video file ",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_stereo_rectified_frames = checked;
              stereo_rectified_frames_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_stereo_rectified_frames;
              return true;
            }
            return false;
          });

  stereo_rectified_frames_filename_ctl =
      add_textbox("Stereo frames file name:",
          "Save rectified frames laid out horizontally into single video file ",
          [this](const QString & value) {
            if ( options_ ) {
              options_->stereo_rectified_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->stereo_rectified_frames_filename.c_str();
              return true;
            }
            return false;
          });

  stereo_rectified_frames_filename_ctl->setPlaceholderText("auto");

  ///

  save_quad_rectified_frames_ctl =
      add_checkbox("Save quad frames",
          "Save debug video with qual layout",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_quad_rectified_frames = checked;
              quad_rectified_frames_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->save_quad_rectified_frames;
              return true;
            }
            return false;
          });

  quad_rectified_frames_filename_ctl =
      add_textbox("Quad frames file name:",
          "Save debug video with qual layout",
          [this](const QString & value) {
            if ( options_ ) {
              options_->quad_rectified_frames_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->quad_rectified_frames_filename.c_str();
              return true;
            }
            return false;
          });

  quad_rectified_frames_filename_ctl->setPlaceholderText("auto");

  ///


  updateControls();
}


void QStereoCalibrationOutputOptions::onupdatecontrols()
{
  Base::onupdatecontrols();

  if( options_ ) {
    //output_directory_ctl->setCurrentPath(options_->output_directory.c_str(), false);
    //calibration_progress_filename_ctl->setEnabled(save_calibration_progress_video_ctl->isChecked());
    //rectified_images_filename_ctl->setEnabled(save_rectified_images_ctl->isChecked());
    //stereo_rectified_frames_filename_ctl->setEnabled(save_stereo_rectified_frames_ctl->isChecked());
    //quad_rectified_frames_filename_ctl->setEnabled(save_quad_rectified_frames_ctl->isChecked());
  }
}
