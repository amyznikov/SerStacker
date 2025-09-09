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

  QExpandableGroupBox(const QString & title, QWidget * view, int stretch = 0, Qt::Alignment alignment = Qt::Alignment(),
      QWidget * parent = nullptr);

  QCheckBox * checkbox() const;
  QVBoxLayout * boxlayout() const;

  void setView(QWidget * view);
  QWidget * view() const;

  void expand();
  void collapse();
  void toggle();

Q_SIGNALS:
  void expanded();
  void collapsed();

protected:
  QWidget * view_ = nullptr;
  QVBoxLayout * layout_ = nullptr;
  QVBoxLayout * frameLayout_ = nullptr;
  QCheckBox * chkBox_ = nullptr;
  QGroupBox * frame_ = nullptr;
};

#endif /* __QExpandableGroupBox_h__ */
