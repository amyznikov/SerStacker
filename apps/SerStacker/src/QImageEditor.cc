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

  connect(&mtfDisplayFunction_, &QImageViewMtfDisplayFunction::updateDisplay,
      this, &ThisClass::updateDisplay);
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
