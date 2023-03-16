/*
 * QRStereoOutputOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QRStereoOutputOptions.h"


QRStereoOutputOptions::QRStereoOutputOptions(QWidget * parent) :
  Base("QRStereoCalibrationOutputOptions", parent)
{
  ///
  form->addRow(output_directory_ctl =
      new QBrowsePathCombo("Output directory:",
          QFileDialog::AcceptSave,
          QFileDialog::AnyFile,
          this));

  output_directory_ctl->setShowDirsOnly(true);


  connect(output_directory_ctl, &QBrowsePathCombo::pathChanged,
      [this] () {
        if ( pipeline_ && !updatingControls() ) {
          pipeline_->set_output_directory(output_directory_ctl->currentPath().toStdString());
          Q_EMIT parameterChanged();
        }
      });

  ///

  save_calibration_config_file_ctl =
      add_checkbox("Save calibration file:",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_calibration_config_file = checked;
              calibration_config_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_calibration_config_file;
              return true;
            }
            return false;
          });

  form->addRow("Calibration file name:",
      calibration_config_filename_ctl =
          new QBrowsePathCombo("",
              QFileDialog::AcceptMode::AcceptSave,
              QFileDialog::AnyFile));

  connect(calibration_config_filename_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( pipeline_ && !updatingControls() ) {
          pipeline_->output_options(). calibration_config_filename =
              calibration_config_filename_ctl->currentPath().toStdString();
          Q_EMIT parameterChanged();
        }
      });

  ///

  save_progress_video_ctl =
      add_checkbox("Save progress frames",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_progress_video = checked;
              progress_video_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_progress_video;
              return true;
            }
            return false;
          });

  progress_video_filename_ctl =
      add_textbox("progress_images_file_name:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().progress_video_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().progress_video_filename.c_str();
              return true;
            }
            return false;
          });

  progress_video_filename_ctl->setPlaceholderText("auto");

  ///

  save_rectified_video_ctl =
      add_checkbox("Save rectified frames",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_rectified_videos = checked;
              left_rectified_video_filename_ctl->setEnabled(checked);
              right_rectified_video_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_rectified_videos;
              return true;
            }
            return false;
          });

  left_rectified_video_filename_ctl =
      add_textbox("Left rectified video filename:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().rectified_video_filenames[0] = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().rectified_video_filenames[0].c_str();
              return true;
            }
            return false;
          });

  left_rectified_video_filename_ctl->setPlaceholderText("auto");

  right_rectified_video_filename_ctl =
      add_textbox("Right rectified video filename:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().rectified_video_filenames[1] = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().rectified_video_filenames[1].c_str();
              return true;
            }
            return false;
          });

  right_rectified_video_filename_ctl->setPlaceholderText("auto");

  ///

  save_stereo_matches_video_ctl =
      add_checkbox("Save match progress video",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_stereo_match_progress_video = checked;
              stereo_match_progress_video_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_stereo_match_progress_video;
              return true;
            }
            return false;
          });

  stereo_matches_video_filename_ctl =
      add_textbox("stereo matches filename:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().stereo_matches_video_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().stereo_matches_video_filename.c_str();
              return true;
            }
            return false;
          });

  stereo_matches_video_filename_ctl->setPlaceholderText("auto");

  ///

  save_motion_poses_ctl =
      add_checkbox("Save motion poses",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_motion_poses = checked;
              motion_poses_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_motion_poses;
              return true;
            }
            return false;
          });

  motion_poses_filename_ctl =
      add_textbox("motion_poses_file_name:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().motion_poses_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().motion_poses_filename.c_str();
              return true;
            }
            return false;
          });

  motion_poses_filename_ctl->setPlaceholderText("auto");

  ///

  save_stereo_match_progress_video_ctl =
      add_checkbox("Save match progress video:",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_stereo_match_progress_video = checked;
              stereo_match_progress_video_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_stereo_match_progress_video;
              return true;
            }
            return false;
          });

  stereo_match_progress_video_filename_ctl =
      add_textbox("match progress filename:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().stereo_match_progress_video_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().stereo_match_progress_video_filename.c_str();
              return true;
            }
            return false;
          });

  stereo_match_progress_video_filename_ctl->setPlaceholderText("auto");

  ///

  updateControls();
}

void QRStereoOutputOptions::set_current_pipeline(const c_regular_stereo_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_regular_stereo_pipeline::sptr & QRStereoOutputOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoOutputOptions::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();

    output_directory_ctl->setCurrentPath(pipeline_->output_directory().c_str(), false);
    calibration_config_filename_ctl->setEnabled(save_calibration_config_file_ctl->isChecked());
    progress_video_filename_ctl->setEnabled(save_progress_video_ctl->isChecked());
    left_rectified_video_filename_ctl->setEnabled(save_rectified_video_ctl->isChecked());
    right_rectified_video_filename_ctl->setEnabled(save_rectified_video_ctl->isChecked());
    stereo_matches_video_filename_ctl->setEnabled(save_stereo_matches_video_ctl->isChecked());
    motion_poses_filename_ctl->setEnabled(save_motion_poses_ctl->isChecked());
    stereo_match_progress_video_filename_ctl->setEnabled(save_stereo_match_progress_video_ctl->isChecked());

    setEnabled(true);
  }
}
