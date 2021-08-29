/*
 * QInPaintSettings.cc
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#include "QInPaintSettings.h"


const QInPaintSettings::ClassFactory QInPaintSettings::classFactory;

QInPaintSettings::QInPaintSettings(const c_inpaint_routine::ptr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  ctlform->addRow(label_ctl = new QLabel("Average pyramid inpaint.\n"
      "No parameters yet"));

  updateControls();
}

void QInPaintSettings::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
  Base::onupdatecontrols();
}
