/*
 * QStackedGroupBox.h
 *
 *  Created on: Oct 8, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QStackedGroupBox_h__
#define __QStackedGroupBox_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/QEnumComboBox.h>

class QStackedGroupBox :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QStackedGroupBox ThisClass;
  typedef QWidget Base;

  QStackedGroupBox(const QString & label, QWidget * parent = nullptr);
  QStackedGroupBox(const QString & label, const c_enum_member * membs, QWidget * parent = nullptr);

  void setupComboboxItems(const c_enum_member * membs);
  void updateSelection();

  void addWidget(int selectorValue, QWidget * widget);
  void setCurrentSelection(int selectorValue);

Q_SIGNALS:
  void currentSelectionChanged(int selectorValue);

protected Q_SLOTS:
  void onComboboxCurrentItemChanged(int index);

protected :
//  QSize sizeHint() const override;
//  QSize minimumSizeHint() const override;

protected:
  QHBoxLayout * _hbox = nullptr;
  QVBoxLayout * _vbox = nullptr;
  QLabel * _label = nullptr;
  QEnumComboBoxBase * _combo = nullptr;
  QStackedWidget * _stack = nullptr;
};

#endif /* __QStackedGroupBox_h__ */
