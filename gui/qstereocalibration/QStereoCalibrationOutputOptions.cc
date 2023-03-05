/*
 * QStereoCalibrationOutputOptions.cc
 *
 *  Created on: Mar 2, 2023
 *      Author: amyznikov
 */

#include "QStereoCalibrationOutputOptions.h"


QStereoCalibrationOutputOptions::QStereoCalibrationOutputOptions(QWidget * parent) :
  Base("QStereoCalibrationOutputOptions", parent)
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

  save_rectified_images_ctl =
      add_checkbox("Save rectified frames",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_rectified_frames = checked;
              rectified_images_file_name_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_rectified_frames;
              return true;
            }
            return false;
          });

  rectified_images_file_name_ctl =
      add_textbox("rectified_images_file_name:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().rectified_frames_file_name = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().rectified_frames_file_name.c_str();
              return true;
            }
            return false;
          });

  ///

  save_stereo_rectified_frames_ctl =
      add_checkbox("Save stereo rectified frames",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_stereo_rectified_frames = checked;
              stereo_rectified_frames_file_name_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_stereo_rectified_frames;
              return true;
            }
            return false;
          });

  stereo_rectified_frames_file_name_ctl =
      add_textbox("stereo rectified frames file name:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().stereo_rectified_frames_file_name = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().stereo_rectified_frames_file_name.c_str();
              return true;
            }
            return false;
          });


  ///

  save_quad_rectified_frames_ctl =
      add_checkbox("Save quad rectified frames",
          [this](bool checked) {
            if ( pipeline_ ) {
              pipeline_->output_options().save_quad_rectified_frames = checked;
              quad_rectified_frames_file_name_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( pipeline_ ) {
              *checked = pipeline_->output_options().save_quad_rectified_frames;
              return true;
            }
            return false;
          });

  quad_rectified_frames_file_name_ctl =
      add_textbox("quad rectified frames file name:",
          [this](const QString & value) {
            if ( pipeline_ ) {
              pipeline_->output_options().quad_rectified_frames_file_name = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( pipeline_ ) {
              *value = pipeline_->output_options().quad_rectified_frames_file_name.c_str();
              return true;
            }
            return false;
          });

  ///


  updateControls();
}

void QStereoCalibrationOutputOptions::set_current_pipeline(const c_stereo_calibration_pipeline::sptr & pipeline)
{
  pipeline_ = pipeline;
  updateControls();
}

const c_stereo_calibration_pipeline::sptr & QStereoCalibrationOutputOptions::current_pipeline() const
{
  return pipeline_;
}

void QStereoCalibrationOutputOptions::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();

    output_directory_ctl->setCurrentPath(pipeline_->output_directory().c_str(), false);
    rectified_images_file_name_ctl->setEnabled(save_rectified_images_ctl->isChecked());
    stereo_rectified_frames_file_name_ctl->setEnabled(save_stereo_rectified_frames_ctl->isChecked());
    quad_rectified_frames_file_name_ctl->setEnabled(save_quad_rectified_frames_ctl->isChecked());

    setEnabled(true);
  }
}
