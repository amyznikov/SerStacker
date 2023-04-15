/*
 * QLogWidget.cc
 *
 *  Created on: Mar 24, 2023
 *      Author: amyznikov
 */

#include "QLogWidget.h"
#include <core/debug.h>

QLogWidget::QLogWidget(QWidget * parent) :
  Base(parent)
{
  QVBoxLayout * vbox =
      new QVBoxLayout(this);

  vbox->setContentsMargins(0, 0, 0, 0);

  vbox->addWidget(textbox_ctl = new QPlainTextEdit(this));
  textbox_ctl->setReadOnly(true);
  textbox_ctl->setWordWrapMode(QTextOption::NoWrap);
  textbox_ctl->document()->setDefaultFont(QFont("Monospace", 11));
  textbox_ctl->setMaximumBlockCount(1000);

  connect(this, &ThisClass::appendText,
      this, &ThisClass::onAppendText,
      Qt::QueuedConnection);


  cf_set_logfunc(&ThisClass::cf_log_func, this);
}


void QLogWidget::setMaxLines(int v)
{
  textbox_ctl->setMaximumBlockCount(v);
}

int QLogWidget::maxLines() const
{
  return textbox_ctl->maximumBlockCount();
}

void QLogWidget::cf_log_func(void * context, const char * msg)
{
  QLogWidget * _this =
      static_cast<QLogWidget * >(context);

  Q_EMIT _this->appendText(msg);
}

void QLogWidget::clear()
{
  textbox_ctl->clear();
}

void QLogWidget::onAppendText(const QString & msg)
{
  if ( textbox_ctl ) {

    if ( msg == "." ) {
      ppmark = true;
      textbox_ctl->moveCursor (QTextCursor::EndOfLine);
      textbox_ctl->insertPlainText(msg);
    }
    else {
      textbox_ctl->moveCursor (QTextCursor::End);

      if ( ppmark ) {
        ppmark = false;
        // textbox_ctl->moveCursor (QTextCursor::StartOfLine);
      }

      textbox_ctl->appendPlainText(msg);
      textbox_ctl->moveCursor (QTextCursor::End);
      //textbox_ctl->moveCursor (QTextCursor::StartOfLine);
      //textbox_ctl->moveCursor (QTextCursor::End);
      textbox_ctl->ensureCursorVisible();
    }
  }
}
