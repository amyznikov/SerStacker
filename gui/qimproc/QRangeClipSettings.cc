/*
 * QRangeClipSettings.cc
 *
 *  Created on: Aug 6, 2021
 *      Author: amyznikov
 */

#include "QRangeClipSettings.h"

const QRangeClipSettings::ClassFactory QRangeClipSettings::classFactory;

QRangeClipSettings::QRangeClipSettings(const c_rangeclip_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  min_ctl = add_numeric_box("min:",
      &c_rangeclip_routine::min,
      &c_rangeclip_routine::set_min);

  max_ctl = add_numeric_box("max:",
      &c_rangeclip_routine::max,
      &c_rangeclip_routine::set_max);
}


void QRangeClipSettings::onupdatecontrols()
{
  Base::onupdatecontrols();

  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    min_ctl->setValue(routine_->min());
    max_ctl->setValue(routine_->max());
    setEnabled(true);
  }
}
