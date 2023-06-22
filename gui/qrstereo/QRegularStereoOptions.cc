/*
 * QRegularStereoOptions.cc
 *
 *  Created on: Mar 28, 2023
 *      Author: amyznikov
 */

#include "QRegularStereoOptions.h"

QRegularStereoOptions::QRegularStereoOptions(QWidget * parent) :
  ThisClass(nullptr, parent)
{
}

QRegularStereoOptions::QRegularStereoOptions(c_regular_stereo * rstereo, QWidget * parent) :
    Base("", parent),
    rstereo_(nullptr)
{
  enable_stereo_rectification_ctl =
      add_checkbox("enable_stereo_rectification",
          "enable_stereo_rectification",
          [this](bool checked) {
            if ( rstereo_ ) {
              rstereo_->set_enable_stereo_rectification(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( rstereo_ ) {
              *checked = rstereo_->enable_stereo_rectification();
              return true;
            }
            return false;
          });

  camera_intrinsics_yml_ctl =
      add_browse_for_path("", "Camera intrinsics YML:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & path) {
            if ( rstereo_ ) {
              rstereo_->set_camera_intrinsics_yml(path.toStdString());
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( rstereo_ ) {
              * path = rstereo_->camera_intrinsics_yml().c_str();
              return true;
            }
            return false;
          });


  camera_extrinsics_yml_ctl =
      add_browse_for_path("", "Camera extrinsics YML:",
          QFileDialog::AcceptOpen,
          QFileDialog::ExistingFile,
          [this](const QString & path) {
            if ( rstereo_ ) {
              rstereo_->set_camera_extrinsics_yml(path.toStdString());
              Q_EMIT parameterChanged();
            }
          },
          [this](QString * path) {
            if ( rstereo_ ) {
              * path = rstereo_->camera_extrinsics_yml().c_str();
              return true;
            }
            return false;
          });


  add_expandable_groupbox("Stereo Matcher",
      stereoMatcherOptions_ctl = new QStereoMatcherOptions());
  connect(stereoMatcherOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  add_expandable_groupbox("Image Processing",
      imageProcessingOptions_ctl = new QRStereoImageProcessingOptions());
  connect(imageProcessingOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  set_rstereo(rstereo);
}

void QRegularStereoOptions::set_rstereo(c_regular_stereo * rstereo)
{
  rstereo_ = rstereo;
  updateControls();
}

c_regular_stereo * QRegularStereoOptions::rstereo() const
{
  return rstereo_;
}

void QRegularStereoOptions::onupdatecontrols()
{
  if ( !rstereo_ ) {
    setEnabled(false);
    stereoMatcherOptions_ctl->set_stereo_matcher(nullptr);
    imageProcessingOptions_ctl->set_options(nullptr);
  }
  else {
    stereoMatcherOptions_ctl->set_stereo_matcher(&rstereo_->stereo_matcher());
    imageProcessingOptions_ctl->set_options(&rstereo_->image_processing_options());
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

void QRegularStereoOptions::updateRunTimeStateControls(bool isRunTime)
{
  camera_intrinsics_yml_ctl->setEnabled(!isRunTime);
  camera_extrinsics_yml_ctl->setEnabled(!isRunTime);
  stereoMatcherOptions_ctl->matcherTypeControl()->setEnabled(!isRunTime);
  imageProcessingOptions_ctl->setEnabled(!isRunTime);
}
