/*
 * QColorSaturationOptions.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#include "QColorSaturationOptions.h"


const QColorSaturationOptions::ClassFactory QColorSaturationOptions::classFactory;

QColorSaturationOptions::QColorSaturationOptions(const RoutinePtr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  scale_ctl = add_numeric_box("Scale:",
      &RoutineType::scale,
      &RoutineType::set_scale);

  updateControls();
}

void QColorSaturationOptions::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    scale_ctl->setValue(routine_->scale());
    setEnabled(true);
  }

  Base::onupdatecontrols();
}
