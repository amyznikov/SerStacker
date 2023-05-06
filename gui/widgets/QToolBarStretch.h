/*
 * QToolBarStretch.h
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QToolBarStretch_h__
#define __QToolBarStretch_h__

#include <QtWidgets/QtWidgets>

class QToolBarStretch:
    public QWidget
{
  Q_OBJECT;
public:
  typedef QToolBarStretch ThisClas;
  typedef QWidget Base;

  QToolBarStretch(QWidget * parent = nullptr);
};

#endif /* __QToolBarStretch_h__ */
