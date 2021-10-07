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
  lksize_ctl = add_numeric_box("lksize:",
      &c_smap_routine::lksize,
      &c_smap_routine::set_lksize);

  scale_size_ctl = add_numeric_box("Scale:",
      &c_smap_routine::scale_size,
      &c_smap_routine::set_scale_size);

  minv_ctl = add_numeric_box("Minv:",
      &c_smap_routine::minv,
      &c_smap_routine::set_minv);


  updateControls();
}

void QSmapSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    lksize_ctl->setValue(routine_->lksize());
    scale_size_ctl->setValue(routine_->scale_size());
    minv_ctl->setValue(routine_->minv());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}

