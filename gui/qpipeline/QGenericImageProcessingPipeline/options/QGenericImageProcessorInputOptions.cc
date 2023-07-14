/*
 * QGenericImageProcessorInputOptions.cc
 *
 *  Created on: Jul 12, 2023
 *      Author: amyznikov
 */

#include "QGenericImageProcessorInputOptions.h"

QGenericImageProcessorInputOptions::QGenericImageProcessorInputOptions(QWidget * parent) :
  Base(parent)
{
  updateControls();
}

void QGenericImageProcessorInputOptions::update_pipeline_controls()
{
  Base::update_pipeline_controls();
}
