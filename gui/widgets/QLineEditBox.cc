/*
 * QLineEditBox.cc
 *
 *  Created on: Oct 7, 2019
 *      Author: amyznikov
 */

#include "QLineEditBox.h"

///////////////////////////////////////////////////////////////////////////////

QLineEditBox::QLineEditBox(QWidget *parent)
  : Base(parent)
{
  construct();
}

QLineEditBox::QLineEditBox(const QString & s, QWidget *parent)
  : Base(s, parent)
{
  construct();
}

void QLineEditBox::construct(void)
{
  connect(this, &Base::editingFinished,
      [this] () {
        if ( Base::text() != previousText ) {
          if ( this->hasFocus() ) {
            previousText = text();
          }
          emit textChanged();
        }
      });
}

void QLineEditBox::focusInEvent(QFocusEvent* e)
{
  previousText = Base::text();
  Base::focusInEvent(e);
}


///////////////////////////////////////////////////////////////////////////////
