/*
 * QStereoMatcherInputOptions.cc
 *
 *  Created on: Jul 13, 2023
 *      Author: amyznikov
 */

#include "QStereoMatcherInputOptions.h"

QStereoMatcherInputOptions::QStereoMatcherInputOptions(QWidget * parent) :
  Base(parent)
{
  updateControls();
}


void QStereoMatcherInputOptions::update_pipeline_controls()
{
  Base::update_pipeline_controls();
}
