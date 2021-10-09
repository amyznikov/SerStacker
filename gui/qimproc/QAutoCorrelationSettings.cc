/*
 * QAutoCorrelationSettings.cc
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#include "QAutoCorrelationSettings.h"

const QAutoCorrelationSettings::ClassFactory QAutoCorrelationSettings::classFactory;

QAutoCorrelationSettings::QAutoCorrelationSettings(const c_auto_correlation_routine::ptr & processor, QWidget * parent)
  : Base(&classFactory, processor, parent)
{
  updateControls();
}

void QAutoCorrelationSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }

  Base::onupdatecontrols();
}
