/*
 * QRStereoCalibrationOutputOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QRStereoCalibrationOutputOptions.h"


QRStereoCalibrationOutputOptions::QRStereoCalibrationOutputOptions(QWidget * parent) :
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

  ///

  save_rectified_video_ctl =
      add_checkbox("Save rectified frames",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_rectified_video = checked;
              rectified_video_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_rectified_video;
              return true;
            }
            return false;
          });

  rectified_video_filename_ctl =
      add_textbox("rectified video filename:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().rectified_video_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().rectified_video_filename.c_str();
              return true;
            }
            return false;
          });

  ///

  save_stereo_matches_video_ctl =
      add_checkbox("Save stereo matches",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_stereo_matches_video = checked;
              stereo_matches_video_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_stereo_matches_video;
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

  ///

  updateControls();
}

void QRStereoCalibrationOutputOptions::set_current_pipeline(const c_rstereo_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_rstereo_calibration_pipeline::sptr & QRStereoCalibrationOutputOptions::current_pipeline() const
{
  return pipeline_;
}

void QRStereoCalibrationOutputOptions::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();

    output_directory_ctl->setCurrentPath(pipeline_->output_directory().c_str(), false);
    progress_video_filename_ctl->setEnabled(save_progress_video_ctl->isChecked());
    rectified_video_filename_ctl->setEnabled(save_rectified_video_ctl->isChecked());
    stereo_matches_video_filename_ctl->setEnabled(save_stereo_matches_video_ctl->isChecked());
    motion_poses_filename_ctl->setEnabled(save_motion_poses_ctl->isChecked());

    setEnabled(true);
  }
}
