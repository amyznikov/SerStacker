/*
 * QFlagsEditBox.h
 *
 *  Created on: Feb 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFlagsEditBox_h__
#define __QFlagsEditBox_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/qsprintf.h>
#include <core/ssprintf.h>

class QFlagsEditBoxBase :
    public QFrame
{
  Q_OBJECT;
public:
  typedef QFlagsEditBoxBase ThisClass;
  typedef QFrame Base;

  QFlagsEditBoxBase(QWidget * parent = nullptr);
  QFlagsEditBoxBase(const c_enum_member * membs, QWidget * parent = nullptr);

  int flags() const;
  void setFlags(int value);

  void setupItems(const c_enum_member * membs);

Q_SIGNALS:
  void flagsChanged(int flags);

protected:
  void updateControls();
  virtual void updateLabelText();
  void onMenuButtonClicked();

protected:
  QHBoxLayout * layout_ = nullptr;
  QLineEdit * editBox_ = nullptr;
  QToolButton * menuButton_ = nullptr;
  QList<QAction *> actions_;

  int flags_ = 0;
  bool updatingControls_ = false;
};

template<class E>
class QFlagsEditBox:
    public QFlagsEditBoxBase
{
public:
  typedef QFlagsEditBox ThisClass;
  typedef QFlagsEditBoxBase Base;
  typedef E ItemType;

  QFlagsEditBox(QWidget * parent = nullptr) :
    Base(parent)
  {
    setupItems(members_of<E>());
  }

protected:
  void updateLabelText() override
  {
    editBox_->setText(qsprintf("0x%0X %s", flags_, flagsToString<E>(flags_).c_str()));
  }

};

#endif /* __QFlagsEditBox_h__ */
