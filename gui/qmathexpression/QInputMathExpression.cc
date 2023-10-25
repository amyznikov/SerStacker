/*
 * QInputMathExpression.cc
 *
 *  Created on: Oct 25, 2023
 *      Author: amyznikov
 */

#include "QInputMathExpression.h"
#include <core/debug.h>

QMathExpressionTextEdit::QMathExpressionTextEdit(QWidget * parent) :
    Base(parent)
{
}

void QMathExpressionTextEdit::focusOutEvent(QFocusEvent *e)
{
  Base::focusOutEvent(e);
  Q_EMIT editFinished();
}

void QMathExpressionTextEdit::keyPressEvent(QKeyEvent *e)
{
  Base::keyPressEvent(e);
  //CF_DEBUG("e->isAccepted=%d", e->isAccepted());

  if ( e->modifiers() == Qt::ControlModifier ) {
    if ( e->key() == Qt::Key_Return ) {
      //CF_DEBUG("Crrl+Return");
      Q_EMIT editFinished();
    }
  }

}


QInputMathExpressionWidget::QInputMathExpressionWidget(QWidget * parent) :
  Base(parent)
{
  QVBoxLayout * box1 = new QVBoxLayout(this);
  QHBoxLayout * box2 = new QHBoxLayout();

  box2->addWidget(apply_ctl = new QPushButton("Apply"));
  box2->addWidget(showFunctions_ctl = new QPushButton("Functions..."));

  box1->addWidget(expressionTextBox_ctl = new QMathExpressionTextEdit(this));
  box1->addLayout(box2);

  connect(apply_ctl, &QPushButton::clicked,
      this, &ThisClass::apply);

  connect(expressionTextBox_ctl, &QMathExpressionTextEdit::editFinished,
      this, &ThisClass::apply);

}

void QInputMathExpressionWidget::setText(const QString & text)
{
  expressionTextBox_ctl->setPlainText(text);
}

QString QInputMathExpressionWidget::text() const
{
  return expressionTextBox_ctl->toPlainText();
}

QInputMathExpressionDialogBox::QInputMathExpressionDialogBox(QWidget * parent) :
    Base(parent)
{
  QVBoxLayout * vbox =
      new QVBoxLayout(this);

  vbox->addWidget(controlWidget_ =
      new QInputMathExpressionWidget(this));

}

void QInputMathExpressionDialogBox::setText(const QString & text)
{
  controlWidget_->setText(text);
}

QString QInputMathExpressionDialogBox::text() const
{
  return controlWidget_->text();
}
