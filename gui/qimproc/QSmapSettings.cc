/*
 * QSmapSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QSmapSettings.h"

const QSmapSettings::ClassFactory QSmapSettings::classFactory;

QSmapSettings::QSmapSettings(const c_smap_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  minv_ctl = add_numeric_box("Minv:", &routine_,
      &c_smap_routine::minv,
      &c_smap_routine::set_minv);

  scale_ctl = add_numeric_box("Scale:", &routine_,
      &c_smap_routine::scale,
      &c_smap_routine::set_scale);

  updateControls();
}

void QSmapSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    minv_ctl->setValue(routine_->minv());
    scale_ctl->setValue(routine_->scale());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}

