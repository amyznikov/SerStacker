/*
 * QEnumComboBox.h
 *
 *  Created on: Dec 12, 2016
 *      Author: amyznikov
 */
#pragma once
#ifndef __QEnumComboBox_h__
#define __QEnumComboBox_h__

#include <QtWidgets/QtWidgets>
#include <core/ssprintf.h>

///////////////////////////////////////////////////////////////////////////////

// QEnumCombobox<> can't emit events
//  because Q_OBJECT templates are not well supported by Qt
class QEnumComboBoxBase :
    public QComboBox
{
  Q_OBJECT;
public:
  typedef QEnumComboBoxBase ThsClass;
  typedef QComboBox Base;

  QEnumComboBoxBase(QWidget * parent = nullptr);
  QEnumComboBoxBase(const c_enum_member * membs, QWidget * parent = nullptr);

  void setupItems(const c_enum_member * membs);

Q_SIGNALS:
  void currentItemChanged(int index);

protected:
  void wheelEvent(QWheelEvent *e) override;

protected:
  QCompleter * completer_ = nullptr;
};


template<class E>
class QEnumComboBox :
    public QEnumComboBoxBase
{

public:
  typedef QEnumComboBox ThisClass;
  typedef QEnumComboBoxBase Base;
  typedef E ItemType;

  QEnumComboBox(QWidget * parent = nullptr) :
    Base(parent)
  {
    setupItems(members_of<E>());
  }

  void setCurrentItem(E value)
  {
    QComboBox::setCurrentIndex(QComboBox::findData((int) (value)));
  }

  E currentItem(void)
  {
    return QComboBox::currentIndex() >= 0 ?
        (E) (QComboBox::currentData().toInt()) :
        (E) (-1);
  }

  void setValue(E value)
  {
    QComboBox::setCurrentIndex(QComboBox::findData((int) (value)));
  }

};



///////////////////////////////////////////////////////////////////////////////
#endif /* __QEnumComboBox_h__ */
