/*
 * QJovianEllipseSettings.cc
 *
 *  Created on: Aug 21, 2022
 *      Author: amyznikov
 */

#include "QJovianEllipseSettings.h"

QJovianEllipseSettings::QJovianEllipseSettings(const c_fit_jovian_ellipse_routine::ptr & processor, QWidget * parent) :
  Base(processor, parent)
{
}

void QJovianEllipseSettings::setupControls()
{
  const c_fit_jovian_ellipse_routine::ptr routine =
      std::dynamic_pointer_cast<c_fit_jovian_ellipse_routine>(processor_);

  settings_ctl =
      add_widget<QJovianEllipseDetectorSettings>();


  display_ctl =
      add_enum_combobox<DisplayType>("Display",
          [this](DisplayType v) {
            const c_fit_jovian_ellipse_routine::ptr routine =
                std::dynamic_pointer_cast<c_fit_jovian_ellipse_routine>(processor_);
            if ( routine ) {
              routine->set_display(v);
              emit parameterChanged();
            }
          });


  connect(settings_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}

void QJovianEllipseSettings::onupdatecontrols()
{
  const c_fit_jovian_ellipse_routine::ptr routine =
      std::dynamic_pointer_cast<c_fit_jovian_ellipse_routine>(processor_);

  if ( !routine ) {
    setEnabled(false);
    settings_ctl->set_jovian_ellipse_detector_options(nullptr);
  }
  else {
    settings_ctl->set_jovian_ellipse_detector_options(&routine->detector()->options());
    display_ctl->setValue(routine->display());
    setEnabled(true);
  }

  Base::onupdatecontrols();
}

