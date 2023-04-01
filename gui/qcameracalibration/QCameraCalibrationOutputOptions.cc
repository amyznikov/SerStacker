/*
 * QCameraCalibrationOutputOptions.cc
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#include "QCameraCalibrationOutputOptions.h"

QCameraCalibrationOutputOptions::QCameraCalibrationOutputOptions(QWidget * parent) :
  Base("QCameraCalibrationOutputOptions", parent)
{
  ///

  form->addRow(output_directory_ctl =
      new QBrowsePathCombo("Output directory:",
          QFileDialog::AcceptSave,
          QFileDialog::Directory,
          this));

  output_directory_ctl->setShowDirsOnly(true);


  connect(output_directory_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( options_ && !updatingControls() ) {
          options_->output_directory = output_directory_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

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

  save_rectified_frames_ctl =
      add_checkbox("Save rectified frames",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_rectified_frames = checked;
              rectified_frames_filename_ctl->setEnabled(checked);
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

  rectified_frames_filename_ctl =
      add_textbox("Rectified video filename:",
          "",
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

  rectified_frames_filename_ctl->setPlaceholderText("auto");

  ///

  save_progress_video_ctl =
      add_checkbox("Save progress video:",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_progress_video = checked;
              progress_video_filename_ctl->setEnabled(checked);
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

  progress_video_filename_ctl =
      add_textbox("progress video filename:",
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

  progress_video_filename_ctl->setPlaceholderText("auto");

  ///

  updateControls();
}

void QCameraCalibrationOutputOptions::set_options(c_camera_calibration_output_options * options)
{
  options_ = options;
  updateControls();
}

c_camera_calibration_output_options * QCameraCalibrationOutputOptions::options() const
{
  return options_;
}

void QCameraCalibrationOutputOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();

    output_directory_ctl->setCurrentPath(options_->output_directory.c_str(), false);
    chessboard_frames_filename_ctl->setEnabled(options_->save_chessboard_frames);
    rectified_frames_filename_ctl->setEnabled(options_->save_rectified_frames);
    progress_video_filename_ctl->setEnabled(options_->save_progress_video);

    setEnabled(true);
  }
}
