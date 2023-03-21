/*
 * QLineEditBox.cc
 *
 *  Created on: Oct 7, 2019
 *      Author: amyznikov
 */

#include "QLineEditBox.h"
#include <core/debug.h>

namespace {

class QCustomLineEdit :
    public QLineEdit
{
public:
  typedef QCustomLineEdit ThisClass;
  typedef QLineEdit Base;

  QCustomLineEdit(const QString & text, QWidget * parent = nullptr) :
    Base(text, parent)
  {
  }

  const QString & previousText() const
  {
    return previousText_;
  }

  void setPreviousText(const QString & s)
  {
    previousText_ = s;
  }

protected:
  void focusInEvent(QFocusEvent *e) override
  {
    previousText_ = Base::text();
    Base::focusInEvent(e);
  }

protected:
  QString previousText_;
};

} /* namespace */

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

  QCustomLineEdit * custom_edit =
      new QCustomLineEdit(s, this);


  custom_edit->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Preferred);

  layout_->addWidget(lineEdit_ = custom_edit, 1000);

  connect(custom_edit, &QLineEdit::editingFinished,
      [this, custom_edit] () {
        if ( custom_edit->text() != custom_edit->previousText() ) {
          if ( this->hasFocus() ) {
            custom_edit->setPreviousText(custom_edit->text());
          }
          Q_EMIT textChanged();
        }
      });

}

///////////////////////////////////////////////////////////////////////////////
