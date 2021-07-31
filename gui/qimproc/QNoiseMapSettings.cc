/*
 * QNoiseMapSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QNoiseMapSettings.h"

const QNoiseMapSettings::ClassFactory QNoiseMapSettings::classFactory;

QNoiseMapSettings::QNoiseMapSettings(const c_noisemap_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  updateControls();
}

void QNoiseMapSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }

  Base::onupdatecontrols();
}
