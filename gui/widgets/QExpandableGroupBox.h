/*
 * QExpandableGroupBox.h
 *
 *  Created on: Mar 29, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QExpandableGroupBox_h__
#define __QExpandableGroupBox_h__

#include <QtWidgets/QtWidgets>

class QExpandableGroupBox:
    public QWidget
{
  Q_OBJECT;
public:
  typedef QExpandableGroupBox ThisClass;
  typedef QWidget Base;

  QExpandableGroupBox(const QString & title, QWidget * view,
      QWidget * parent = Q_NULLPTR);

  QWidget * view() const;

//  void expand();
//  void collapse();
//  void toggle();


protected:
  QWidget * view_ = Q_NULLPTR;
  QFormLayout * layout_ = Q_NULLPTR;
  QCheckBox * chkBox_ = Q_NULLPTR;
  QGroupBox * gBox_ = Q_NULLPTR;
};

#endif /* __QExpandableGroupBox_h__ */
