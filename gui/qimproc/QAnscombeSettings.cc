/*
 * QAnscombeSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QAnscombeSettings.h"

const QAnscombeSettings::ClassFactory QAnscombeSettings::classFactory;

QAnscombeSettings::QAnscombeSettings(const c_anscombe_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  method_ctl = add_enum_combobox<QAnscombeMethodCombo>("Method:",
      &c_anscombe_routine::method,
      &c_anscombe_routine::set_method);

  updateControls();
}

void QAnscombeSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    method_ctl->setCurrentItem(routine_->method());
    setEnabled(true);
  }

  Base::onupdatecontrols();
}
