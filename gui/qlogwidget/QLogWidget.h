/*
 * QLogWidget.h
 *
 *  Created on: Mar 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLogWidget_h__
#define __QLogWidget_h__

#include <QtWidgets/QtWidgets>

class QLogWidget:
    public QWidget
{
  Q_OBJECT;
public:
  typedef QLogWidget ThisClass;
  typedef QWidget Base;

  QLogWidget(QWidget * parent = nullptr);

  void setMaxLines(int v);
  int maxLines() const;

  void clear();

Q_SIGNALS:
  void appendText(const QString & text/*, QPrivateSignal **/);

protected Q_SLOTS:
  void onAppendText(const QString & text);

protected:
  static void cf_log_func(void * context, const char * msg);

protected:
  QPlainTextEdit * textbox_ctl = nullptr;
  bool ppmark = false;
};

#endif /* __QLogWidget_h__ */
