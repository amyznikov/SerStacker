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
inline QLineEditBox * add_editbox(QFormLayout * form, const char * parameter_name, _Calable && slot)
{
  QLineEditBox * ctl = new QLineEditBox();
  form->addRow(parameter_name, ctl);
  QObject::connect(ctl, &QLineEditBox::textChanged, slot);

  return ctl;
}

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

inline QGroupBox * add_expandable_groupbox(QFormLayout * form, const QString & title, QWidget * ctl)
{
#define __STYLE_TEXT(x) #x
  static const char style[] = __STYLE_TEXT(
      QCheckBox {
          spacing: 15px;
      }
      QCheckBox::indicator {
          width: 13px;
          height: 13px;
      }
      QCheckBox::indicator:unchecked {
          image: url(:/gui/icons/double-arrow-right.png);
      }
      QCheckBox::indicator:checked {
          image: url(:/gui/icons/double-arrow-up.png);
      }
  );
#undef __STYLE_TEXT


  QCheckBox * chkBox = new QCheckBox(title);
  chkBox->setStyleSheet(style);
  form->addRow(chkBox);

  QGroupBox * gb = new QGroupBox();
  form->addRow(gb);
  (new QVBoxLayout(gb))->addWidget(ctl);
  gb->setVisible(false);

  QObject::connect(chkBox, &QCheckBox::stateChanged,
      [gb](int state) {
        gb->setVisible(state == Qt::Checked);
      });

  return gb;
}

#endif /* __adddock_h__ */
