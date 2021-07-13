/*
 * QToolbarSpacer.h
 *
 *  Created on: Mar 8, 2020
 *      Author: amyznikov
 */

#ifndef __QToolbarSpacer_h__
#define __QToolbarSpacer_h__

#include <QtWidgets/QtWidgets>

class QToolbarSpacer
    : public QWidget
{
public:
  typedef QToolbarSpacer ThisClass;
  typedef QWidget Base;

  QToolbarSpacer(QWidget * parent = Q_NULLPTR);
};

#endif /* __QToolbarSpacer_h__ */
