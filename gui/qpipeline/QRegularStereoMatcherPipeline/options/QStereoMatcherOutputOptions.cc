/*
 * QStereoMatcherOutputOptions.cc
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#include "QStereoMatcherOutputOptions.h"

QStereoMatcherOutputOptions::QStereoMatcherOutputOptions(QWidget * parent) :
  Base(parent)
{
  ///
  save_progress_video_ctl =
      add_checkbox("Save progress frames",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_progress_video = checked;
              progress_video_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_) {
              *checked = options_->save_progress_video;
              return true;
            }
            return false;
          });

  progress_video_filename_ctl =
      add_textbox("progress images file name:",
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

  save_depthmaps_ctl =
      add_checkbox("Save depthmaps",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_depthmaps = checked;
              depthmaps_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_) {
              *checked = options_->save_depthmaps;
              return true;
            }
            return false;
          });

  depthmaps_filename_ctl =
      add_textbox("depthmaps file name:",
          "",
          [this](const QString & value) {
            if ( options_ ) {
              options_->depthmap_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->depthmap_filename.c_str();
              return true;
            }
            return false;
          });

  depthmaps_filename_ctl->setPlaceholderText("auto");

  ///

  //QLineEditBox * cloud3d_image_filename_ctl = nullptr;

  save_cloud3d_image_ctl =
      add_checkbox("Save 3D images",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_cloud3d_image = checked;
              cloud3d_image_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_) {
              *checked = options_->save_cloud3d_image;
              return true;
            }
            return false;
          });

  cloud3d_image_filename_ctl =
      add_textbox("3D images name:",
          "",
          [this](const QString & value) {
            if ( options_ ) {
              options_->cloud3d_image_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->cloud3d_image_filename.c_str();
              return true;
            }
            return false;
          });

  cloud3d_image_filename_ctl->setPlaceholderText("auto");

  ///

  save_cloud3d_ply_ctl =
      add_checkbox("Save 3D ply",
          "",
          [this](bool checked) {
            if ( options_ ) {
              options_->save_cloud3d_ply = checked;
              cloud3d_ply_filename_ctl->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_) {
              *checked = options_->save_cloud3d_ply;
              return true;
            }
            return false;
          });

  cloud3d_ply_filename_ctl =
      add_textbox("3D ply  file name:",
          "",
          [this](const QString & value) {
            if ( options_ ) {
              options_->cloud3d_ply_filename = value.toStdString();
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * value) {
            if ( options_ ) {
              *value = options_->cloud3d_ply_filename.c_str();
              return true;
            }
            return false;
          });

  cloud3d_ply_filename_ctl->setPlaceholderText("auto");

  updateControls();
}

void QStereoMatcherOutputOptions::onupdatecontrols()
{
  Base::onupdatecontrols();

  if( options_ ) {
    //    save_progress_video_ctl->setChecked(options_->save_progress_video);
    //    progress_video_filename_ctl->setEnabled(save_progress_video_ctl->isChecked());
    //
    //    save_depthmaps_ctl->setChecked(options_->save_depthmaps);
    //    depthmaps_filename_ctl->setEnabled(save_depthmaps_ctl->isChecked());
  }
}