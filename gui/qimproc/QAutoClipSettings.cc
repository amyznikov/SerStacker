/*
 * QAutoClipSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QAutoClipSettings.h"

const QAutoClipSettings::ClassFactory QAutoClipSettings::classFactory;

QAutoClipSettings::QAutoClipSettings(const c_autoclip_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  lclip_ctl = add_numeric_box("lclip [%]:", &routine_,
      &c_autoclip_routine::lclip,
      &c_autoclip_routine::set_lclip);

  hclip_ctl = add_numeric_box("hclip [%]:", &routine_,
      &c_autoclip_routine::hclip,
      &c_autoclip_routine::set_hclip);

  updateControls();
}

void QAutoClipSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    lclip_ctl->setValue(routine_->lclip());
    hclip_ctl->setValue(routine_->hclip());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}
