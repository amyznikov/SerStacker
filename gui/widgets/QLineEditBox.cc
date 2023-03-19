/*
 * QLineEditBox.cc
 *
 *  Created on: Oct 7, 2019
 *      Author: amyznikov
 */

#include "QLineEditBox.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////

QLineEditBox::QLineEditBox(QWidget *parent) :
  ThisClass("", parent)
{
}

QLineEditBox::QLineEditBox(const QString & s, QWidget *parent) :
    Base(parent)
{
  layout_ = new QHBoxLayout(this);
  layout_->setContentsMargins(0, 0, 0, 0);

  layout_->addWidget(lineEdit_ = new QLineEdit(s, this), 10000);

  connect(lineEdit_, &QLineEdit::editingFinished,
      [this] () {
        if ( lineEdit_->text() != previousText ) {
          if ( this->hasFocus() ) {
            previousText = lineEdit_->text();
          }
          Q_EMIT textChanged();
        }
      });

}


void QLineEditBox::focusInEvent(QFocusEvent* e)
{
  CF_DEBUG("focusInEvent");
  previousText = lineEdit_->text();
  Base::focusInEvent(e);
}


///////////////////////////////////////////////////////////////////////////////
