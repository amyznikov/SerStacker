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
class QEnumComboBoxBase
  : public QComboBox
{
  Q_OBJECT;
public:
  typedef QComboBox Base;
  QEnumComboBoxBase(QWidget * parent = 0)
    : Base(parent) {
    connect(this, SIGNAL(currentIndexChanged(int)),
        this,  SIGNAL(currentItemChanged(int)));
  }
signals:
  void currentItemChanged(int index);
};


template<class E>
class QEnumComboBox
  : public QEnumComboBoxBase
{

public:
  typedef QEnumComboBox ThisClass;
  typedef QEnumComboBoxBase Base;
  typedef E ItemType;

  QEnumComboBox(QWidget * parent)
    : Base(parent)
  {
    setupItems();
  }

  void setCurrentItem(E value) {
    QComboBox::setCurrentIndex(QComboBox::findData((int) (value)));
  }

  E currentItem(void) {
    return QComboBox::currentIndex() >= 0 ? (E) (QComboBox::currentData().toInt()) : (E) (-1);
  }

protected:
  void setupItems()
  {
    const c_enum_member * membs =
        members_of<E>();

    if ( membs ) {
      while ( membs->name && *membs->name ) {
        QComboBox::addItem(membs->name, (int) (membs->value));
        ++membs;
      }
    }
  }

};


template<class T, class E>
inline bool applySetting(const QString & optName, QEnumComboBox<E> * combo, T * obj,  bool (T::*fn)(E))
{
  E v = combo->currentItem();
  if ( !(obj->*fn)(v) ) {
    QMessageBox::critical(combo, "ERROR", QString("Invalid %1 specified").arg(optName));
    combo->setFocus();
    return false;
  }
  return true;
}


///////////////////////////////////////////////////////////////////////////////
#endif /* __QEnumComboBox_h__ */
