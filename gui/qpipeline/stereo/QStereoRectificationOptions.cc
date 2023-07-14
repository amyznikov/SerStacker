/*
 * QStereoRectificationOptions.cc
 *
 *  Created on: Jul 9, 2023
 *      Author: amyznikov
 */

#include "QStereoRectificationOptions.h"

QStereoRectificationOptions::QStereoRectificationOptions(QWidget * parent) :
  Base("", parent)
{
  enable_stereo_rectification_ctl =
      add_checkbox("enable_stereo_rectification",
          "enable_stereo_rectification",
          [this](bool checked) {
            if ( options_ ) {
              options_->set_enabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->enabled();
              return true;
            }
            return false;
          });

  camera_intrinsics_yml_ctl =
      add_browse_for_path("", "Camera intrinsics YML:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & path) {
            if ( options_ ) {
              options_->set_camera_intrinsics_yml(path.toStdString());
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( options_ ) {
              * path = options_->camera_intrinsics_yml().c_str();
              return true;
            }
            return false;
          });


  camera_extrinsics_yml_ctl =
      add_browse_for_path("", "Camera extrinsics YML:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & path) {
            if ( options_ ) {
              options_->set_camera_extrinsics_yml(path.toStdString());
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( options_ ) {
              * path = options_->camera_extrinsics_yml().c_str();
              return true;
            }
            return false;
          });

  updateControls();
}

void QStereoRectificationOptions::set_rectification_options(c_stereo_rectification_options * options)
{
  options_ = options;
  updateControls();
}

c_stereo_rectification_options * QStereoRectificationOptions::rectification_options() const
{
  return options_;
}

void QStereoRectificationOptions::onupdatecontrols()
{
  if( !options_ ) {
    setEnabled(false);
  }
  else {
    Base::populatecontrols();
    setEnabled(true);
  }
}
