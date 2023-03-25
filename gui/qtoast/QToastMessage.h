/*
 * QToastMessage.h
 *
 *  Created on: Mar 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QToastMessage_h__
#define __QToastMessage_h__

#include <QtWidgets/QtWidgets>

class QToastMessage:
    public QWidget
{
  Q_OBJECT;
public:
  QToastMessage();

  static void information(QWidget * parent, const QString & msg);
  static void warning(QWidget * parent, const QString & msg);
  static void critical(QWidget * parent, const QString & msg);

protected:
  static QToastMessage * singleton(QWidget * parent);
  QGraphicsOpacityEffect * effect = nullptr;
  QPropertyAnimation * fadeInAnimation = nullptr;
  QPropertyAnimation * fadeOutAnimation = nullptr;
};

#endif /* __QToastMessage_h__ */
