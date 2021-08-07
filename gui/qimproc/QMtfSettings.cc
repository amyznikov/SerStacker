/*
 * QMtfSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QMtfSettings.h"

const QMtfSettings::ClassFactory QMtfSettings::classFactory;

QMtfSettings::QMtfSettings(const c_mtf_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  mtf_ctl = add_widget("", new QMtfControl(this));

  updateControls();

  connect(mtf_ctl, &QMtfControl::mtfChanged,
      this, &ThisClass::parameterChanged);
}

void QMtfSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
    mtf_ctl->setMtf(nullptr);
  }
  else {
    mtf_ctl->setMtf(routine_->mtf());
    setEnabled(true);
  }

  Base::onupdatecontrols();
}
