/*
 * QImageEditor.cc
 *
 *  Created on: May 2, 2022
 *      Author: amyznikov
 */

#include "QImageEditor.h"

namespace qserstacker {


QImageEditor::QImageEditor(QWidget * parent) :
  Base(parent),
  mtfDisplayFunction_(this, "QImageEditor")
{
  Base::setDisplayFunction(&mtfDisplayFunction_);

  connect(&mtfDisplayFunction_, &QImageViewMtfDisplayFunction::parameterChanged,
      this, &ThisClass::updateDisplay);

  connect(this, &ThisClass::displayImageChanged,
      &mtfDisplayFunction_, &QMtfDisplay::displayImageChanged,
      Qt::QueuedConnection);

  scene()->setBackgroundBrush(Qt::darkGray);
}

QImageViewMtfDisplayFunction * QImageEditor::mtfDisplayFunction()
{
  return &mtfDisplayFunction_;
}

const QImageViewMtfDisplayFunction * QImageEditor::mtfDisplayFunction() const
{
  return &mtfDisplayFunction_;
}

} /* namespace qserstacker */
