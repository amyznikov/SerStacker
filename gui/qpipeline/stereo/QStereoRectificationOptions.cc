/*
 * QStereoRectificationOptions.cc
 *
 *  Created on: Jul 9, 2023
 *      Author: amyznikov
 */

#include "QStereoRectificationOptions.h"

QStereoRectificationOptions::QStereoRectificationOptions(QWidget * parent) :
  Base(parent)
{
  enable_stereo_rectification_ctl =
      add_checkbox("enable_stereo_rectification",
          "enable_stereo_rectification",
          [this](bool checked) {
            if ( _opts ) {
              _opts->set_enabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              *checked = _opts->enabled();
              return true;
            }
            return false;
          });

  camera_intrinsics_yml_ctl =
      add_browse_for_path("", "Camera intrinsics YML:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & path) {
            if ( _opts ) {
              _opts->set_camera_intrinsics_yml(path.toStdString());
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( _opts ) {
              * path = _opts->camera_intrinsics_yml().c_str();
              return true;
            }
            return false;
          });


  camera_extrinsics_yml_ctl =
      add_browse_for_path("", "Camera extrinsics YML:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & path) {
            if ( _opts ) {
              _opts->set_camera_extrinsics_yml(path.toStdString());
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( _opts ) {
              * path = _opts->camera_extrinsics_yml().c_str();
              return true;
            }
            return false;
          });

  updateControls();
}

//void QStereoRectificationOptions::set_rectification_options(c_stereo_rectification_options * options)
//{
//  _opts = options;
//  updateControls();
//}
//
//c_stereo_rectification_options * QStereoRectificationOptions::rectification_options() const
//{
//  return _opts;
//}

//void QStereoRectificationOptions::onupdatecontrols()
//{
//  if( !_opts ) {
//    setEnabled(false);
//  }
//  else {
//    Base::populatecontrols();
//    setEnabled(true);
//  }
//}
