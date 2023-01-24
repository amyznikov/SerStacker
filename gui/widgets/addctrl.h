/*
 * addctrl.h
 *
 *  Created on: Jan 9, 2019
 *      Author: amyznikov
 */

#ifndef __adddock_h__
#define __adddock_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/QEnumComboBox.h>
#include <gui/widgets/QLineEditBox.h>

QDockWidget * addDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QWidget * clientWidget,
    QMenu * viewMenu = Q_NULLPTR);

template<class _Calable>
inline QLineEditBox * insert_editbox(QFormLayout * form, int row, const char * parameter_name, _Calable && slot)
{
  QLineEditBox * ctl = new QLineEditBox();
  form->insertRow(row, parameter_name, ctl);
  QObject::connect(ctl, &QLineEditBox::textChanged, slot);
  return ctl;
}

template<class _Calable>
inline QNumberEditBox * add_numeric_box(QFormLayout * form, const char * parameter_name, _Calable && slot)
{
  QNumberEditBox * ctl = new QNumberEditBox();
  form->addRow(parameter_name, ctl);
  QObject::connect(ctl, &QLineEditBox::textChanged, slot);

  return ctl;
}

template<class _Calable>
inline QCheckBox * add_checkbox(QFormLayout * form, const char * parameter_name, _Calable && slot)
{
  QCheckBox * ctl = new QCheckBox(parameter_name);
  form->addRow(ctl);
  QObject::connect(ctl, &QCheckBox::stateChanged, slot);
  return ctl;
}

template<class _Calable>
inline QCheckBox * insert_checkbox(QFormLayout * form, int row, const char * parameter_name, _Calable && slot)
{
  QCheckBox * ctl = new QCheckBox(parameter_name);
  form->insertRow(row, ctl);
  QObject::connect(ctl, &QCheckBox::stateChanged, slot);
  return ctl;
}




template<class _Calable>
inline void add_combobox(QFormLayout * form, const char * parameter_name, QEnumComboBoxBase * ctl, _Calable && slot)
{
  form->addRow(parameter_name, ctl);
  QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged, slot);
}

template<class _Calable>
inline void insert_combobox(QFormLayout * form, int row, const char * parameter_name, QEnumComboBoxBase * ctl, _Calable && slot)
{
  form->insertRow(row, parameter_name, ctl);
  QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged, slot);
}




template<class _Calable>
inline QComboBox * add_combobox(QFormLayout * form, const char * parameter_name, _Calable && slot)
{
  QComboBox * ctl =  new QComboBox();
  form->addRow(parameter_name, ctl);
  QObject::connect(ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), slot);
  return ctl;
}

template<class _Calable>
inline QComboBox * insert_combobox(QFormLayout * form, int row, const char * parameter_name, _Calable && slot)
{
  QComboBox * ctl =  new QComboBox();
  form->insertRow(row, parameter_name, ctl);
  QObject::connect(ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), slot);
  return ctl;
}



template<class _Calable>
inline QPushButton * add_pushbutton(QFormLayout * form, const char * command_name, _Calable && slot)
{
  QPushButton * ctl = new QPushButton(command_name);
  form->addRow(ctl);
  QObject::connect(ctl, &QPushButton::clicked, slot);
  return ctl;
}

#endif /* __adddock_h__ */
