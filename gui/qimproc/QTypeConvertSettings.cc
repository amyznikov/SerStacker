/*
 * QTypeConvertSettings.cc
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#include "QTypeConvertSettings.h"

const QTypeConvertSettings::ClassFactory QTypeConvertSettings::classFactory;

QTypeConvertSettings::QTypeConvertSettings(const c_type_convert_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  ddept_ctl = add_enum_combobox<QDDEPTHCombo>("DDEPTH",
      &RoutineType::ddepth,
      &RoutineType::set_ddepth);

  auto_scale_ctl = add_checkbox("Auto scale",
      &RoutineType::auto_scale,
      &RoutineType::set_auto_scale,
      [this]() {
        alpha_ctl->setEnabled(!routine_->auto_scale());
        beta_ctl->setEnabled(!routine_->auto_scale());
      });

  alpha_ctl = add_numeric_box("alpha:",
      &RoutineType::alpha,
      &RoutineType::set_alpha);

  beta_ctl = add_numeric_box("beta:",
      &RoutineType::beta,
      &RoutineType::set_beta);

}

void QTypeConvertSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {

    ddept_ctl->setCurrentItem(routine_->ddepth());
    auto_scale_ctl->setChecked(routine_->auto_scale());
    alpha_ctl->setValue(routine_->alpha());
    beta_ctl->setValue(routine_->beta());
    alpha_ctl->setEnabled(!routine_->auto_scale());
    beta_ctl->setEnabled(!routine_->auto_scale());

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
