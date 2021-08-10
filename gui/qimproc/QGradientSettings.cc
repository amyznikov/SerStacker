/*
 * QGradientSettings.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "QGradientSettings.h"

const QGradientSettings::ClassFactory QGradientSettings::classFactory;

QGradientSettings::QGradientSettings(const c_gradient_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  updateControls();
}

void QGradientSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }

  Base::onupdatecontrols();
}
