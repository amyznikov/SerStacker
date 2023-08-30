/*
 * QStereoMatcherOptions.cc
 *
 *  Created on: Mar 28, 2023
 *      Author: amyznikov
 */

#include "QStereoMatcherOptions.h"

QStereoMatcherOptions::QStereoMatcherOptions(QWidget * parent) :
    Base("", parent)
{
  enabled_ctl =
      add_checkbox("Enable stereo matcher",
          "",
          [this](bool checked) {
            if ( stereo_matcher_ && stereo_matcher_->enabled() != checked ) {
              stereo_matcher_->set_enabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( stereo_matcher_ ) {
              * checked = stereo_matcher_->enabled();
              return true;
            }
            return false;
          });


  matcher_type_ctl =
      add_enum_combobox<stereo_matcher_type>("Stereo Matcher:",
          "Current stereo matcher",
          [this](stereo_matcher_type value) {
            if ( stereo_matcher_ && stereo_matcher_->matcher_type() != value ) {
              stereo_matcher_->set_matcher_type(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](stereo_matcher_type * value) {
            if ( stereo_matcher_ ) {
              * value = stereo_matcher_->matcher_type();
              return true;
            }
            return false;
          });

  add_expandable_groupbox("StereoBM Options",
      stereoBMOptions_ctl = new QStereoBMOptions());

  connect(stereoBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( stereo_matcher_ ) {
          stereo_matcher_->updateStereoBMOptions();
        }
        Q_EMIT parameterChanged();
      });

  add_expandable_groupbox("StereoSGBM Options",
      stereoSGBMOptions_ctl = new QStereoSGBMOptions());

  connect(stereoSGBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( stereo_matcher_ ) {
          stereo_matcher_->updateStereoSGBMOptions();
        }
        Q_EMIT parameterChanged();
      });


  add_expandable_groupbox("ScaleSweep Options",
      scaleSweepOptions_ctl = new QScaleSweepOptions());

  connect(scaleSweepOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( stereo_matcher_ ) {
          stereo_matcher_->updateScaleSweepOptions();
        }
        Q_EMIT parameterChanged();
      });


#if HAVE_OpenCV_stereo

  add_expandable_groupbox("QuasiDenseStereo Options",
      quasiDenseStereoOptions_ctl = new QQuasiDenseStereoOptions());

  connect(quasiDenseStereoOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( stereo_matcher_ ) {
          stereo_matcher_->updateQuasiDenseStereoOptions();
        }
        Q_EMIT parameterChanged();
      });

  add_expandable_groupbox("StereoBinaryBM Options",
      stereoBinaryBMOptions_ctl = new QStereoBinaryBMOptions());

  connect(stereoBinaryBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( stereo_matcher_ ) {
          stereo_matcher_->updateStereoBinaryBMOptions();
        }
        Q_EMIT parameterChanged();
      });

  add_expandable_groupbox("StereoBinarySGBM Options",
      stereoBinarySGBMOptions_ctl = new QStereoBinarySGBMOptions());

  connect(stereoBinarySGBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( stereo_matcher_ ) {
          stereo_matcher_->updateStereoBinarySGBMOptions();
        }
        Q_EMIT parameterChanged();
      });

#endif // HAVE_OpenCV_stereo

  updateControls();
}

void QStereoMatcherOptions::set_stereo_matcher(c_regular_stereo_matcher * stereo_matcher)
{
  stereo_matcher_ = stereo_matcher;
  updateControls();
}

c_regular_stereo_matcher* QStereoMatcherOptions::stereo_matcher() const
{
  return stereo_matcher_;
}

void QStereoMatcherOptions::onupdatecontrols()
{
  if( !stereo_matcher_ ) {
    setEnabled(false);
  }
  else {
    stereoBMOptions_ctl->set_options(&stereo_matcher_->StereoBMOptions());
    stereoSGBMOptions_ctl->set_options(&stereo_matcher_->StereoSGBMOptions());
    scaleSweepOptions_ctl->set_options(&stereo_matcher_->ScaleSweepOptions());

#if HAVE_OpenCV_stereo
    quasiDenseStereoOptions_ctl->set_options(&stereo_matcher_->quasiDenseStereoOptions());
    stereoBinarySGBMOptions_ctl->set_options(&stereo_matcher_->StereoBinarySGBMOptions());
    stereoBinaryBMOptions_ctl->set_options(&stereo_matcher_->StereoBinaryBMOptions());
#endif // HAVE_OpenCV_stereo

    Base::onupdatecontrols();
    setEnabled(true);
  }

}

QEnumComboBox<stereo_matcher_type> * QStereoMatcherOptions::matcherTypeControl() const
{
  return matcher_type_ctl;
}

