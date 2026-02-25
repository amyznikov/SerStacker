/*
 * QStereoMatcherOptions.cc
 *
 *  Created on: Mar 28, 2023
 *      Author: amyznikov
 */

#include "QStereoMatcherOptions.h"

QStereoMatcherOptions::QStereoMatcherOptions(QWidget * parent) :
    Base(parent)
{
  enabled_ctl =
      add_checkbox("Enable stereo matcher",
          "",
          [this](bool checked) {
            if ( _opts && _opts->enabled() != checked ) {
              _opts->set_enabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->enabled();
              return true;
            }
            return false;
          });


  matcher_type_ctl =
      add_enum_combobox<stereo_matcher_type>("Stereo Matcher:",
          "Current stereo matcher",
          [this](stereo_matcher_type value) {
            if ( _opts && _opts->matcher_type() != value ) {
              _opts->set_matcher_type(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](stereo_matcher_type * value) {
            if ( _opts ) {
              * value = _opts->matcher_type();
              return true;
            }
            return false;
          });

  add_expandable_groupbox("StereoBM Options",
      stereoBMOptions_ctl = new QStereoBMOptions());

  QObject::connect(stereoBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( _opts ) {
          _opts->updateStereoBMOptions();
          Q_EMIT parameterChanged();
        }
      });

  add_expandable_groupbox("StereoSGBM Options",
      stereoSGBMOptions_ctl = new QStereoSGBMOptions());

  QObject::connect(stereoSGBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( _opts ) {
          _opts->updateStereoSGBMOptions();
          Q_EMIT parameterChanged();
        }
      });


  add_expandable_groupbox("ScaleSweep Options",
      scaleSweepOptions_ctl = new QScaleSweepOptions());

  QObject::connect(scaleSweepOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( _opts ) {
          _opts->updateScaleSweepOptions();
          Q_EMIT parameterChanged();
        }
      });


#if HAVE_OpenCV_stereo

  add_expandable_groupbox("QuasiDenseStereo Options",
      quasiDenseStereoOptions_ctl = new QQuasiDenseStereoOptions());

  QObject::connect(quasiDenseStereoOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( _opts ) {
          _opts->updateQuasiDenseStereoOptions();
          Q_EMIT parameterChanged();
        }
      });

  add_expandable_groupbox("StereoBinaryBM Options",
      stereoBinaryBMOptions_ctl = new QStereoBinaryBMOptions());

  QObject::connect(stereoBinaryBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( _opts ) {
          _opts->updateStereoBinaryBMOptions();
          Q_EMIT parameterChanged();
        }
      });

  add_expandable_groupbox("StereoBinarySGBM Options",
      stereoBinarySGBMOptions_ctl = new QStereoBinarySGBMOptions());

  QObject::connect(stereoBinarySGBMOptions_ctl, &QSettingsWidget::parameterChanged,
      [this]() {
        if ( _opts ) {
          _opts->updateStereoBinarySGBMOptions();
          Q_EMIT parameterChanged();
        }
      });

#endif // HAVE_OpenCV_stereo

  QObject::connect(this, &ThisClass::populatecontrols,
      [this]() {
        stereoBMOptions_ctl->setOpts(_opts? &_opts->StereoBMOptions() : nullptr);
        stereoSGBMOptions_ctl->setOpts(_opts? &_opts->StereoSGBMOptions() : nullptr);
        scaleSweepOptions_ctl->setOpts(_opts? &_opts->ScaleSweepOptions() : nullptr);
    #if HAVE_OpenCV_stereo
        quasiDenseStereoOptions_ctl->setOpts(_opts? &_opts->quasiDenseStereoOptions() : nullptr);
        stereoBinarySGBMOptions_ctl->setOpts(_opts? &_opts->StereoBinarySGBMOptions() : nullptr);
        stereoBinaryBMOptions_ctl->setOpts(_opts? &_opts->StereoBinaryBMOptions() : nullptr);
    #endif // HAVE_OpenCV_stereo
    });


  updateControls();
}
//void QStereoMatcherOptions::onupdatecontrols()
//{
//  if( !_opts ) {
//    setEnabled(false);
//  }
//  else {
//    stereoBMOptions_ctl->set_options(&_opts->StereoBMOptions());
//    stereoSGBMOptions_ctl->set_options(&_opts->StereoSGBMOptions());
//    scaleSweepOptions_ctl->set_options(&_opts->ScaleSweepOptions());
//
//#if HAVE_OpenCV_stereo
//    quasiDenseStereoOptions_ctl->set_options(&_opts->quasiDenseStereoOptions());
//    stereoBinarySGBMOptions_ctl->set_options(&_opts->StereoBinarySGBMOptions());
//    stereoBinaryBMOptions_ctl->set_options(&_opts->StereoBinaryBMOptions());
//#endif // HAVE_OpenCV_stereo
//
//    Base::onupdatecontrols();
//    setEnabled(true);
//  }
//
//}

QEnumComboBox<stereo_matcher_type> * QStereoMatcherOptions::matcherTypeControl() const
{
  return matcher_type_ctl;
}

