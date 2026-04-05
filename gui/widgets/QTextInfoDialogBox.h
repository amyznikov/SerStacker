/*
 * QTextInfoDialogBox.h
 *
 *  Created on: Apr 4, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QTextInfoDialogBox_h__
#define __QTextInfoDialogBox_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/createAction.h>

class QTextInfoDialogBox :
    public QDialog
{
public:
  typedef QTextInfoDialogBox ThisClass;
  typedef QDialog Base;

  static void show(const QString & title, const QString & text, QWidget * parent = nullptr);

protected:
  QTextInfoDialogBox(QWidget * parent = nullptr);
  void hideEvent(QHideEvent *event) override;

protected:
  static ThisClass * _singleton;
  QToolBar * toolbar_ctl = nullptr;
  QTextBrowser * textbox_ctl = nullptr;
  QLineEditBox * searchText_ctl = nullptr;
  QAction * searchNext_ctl = nullptr;
  QByteArray _geometry;
};

#endif /* __QTextInfoDialogBox_h__ */
