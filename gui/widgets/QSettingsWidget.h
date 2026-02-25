/*
 * QSettingsWidget.h
 *
 *  Created on: Jan 11, 2019
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSettingsWidget_h__
#define __QSettingsWidget_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/settings.h>
#include <gui/widgets/QEnumComboBox.h>
#include <gui/widgets/QFlagsEditBox.h>
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/QExpandableGroupBox.h>
#include <gui/widgets/QSliderSpinBox.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <gui/widgets/QFFmpegOptionsControl.h>
#include <gui/widgets/QColorPickerButton.h>
#include <gui/widgets/QDataAnnotationSelectorCtrl.h>
#include <gui/qmathexpression/QInputMathExpression.h>
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <type_traits>
#include <functional>
#include <mutex>
#include <core/debug.h>


class QSettingsWidget :
    public QFrame
{
  Q_OBJECT;
public:
  typedef QSettingsWidget ThisClass;
  typedef QFrame Base;

  struct c_mutex_lock
  {
    QSettingsWidget * _this;
    c_mutex_lock(QSettingsWidget * ths) : _this(ths) { _this->lock(); }
    ~c_mutex_lock() { _this->unlock(); }
  };

  struct c_update_controls_lock
  {
    ThisClass * _this;
    c_update_controls_lock(ThisClass * obj) : _this(obj) { _this->setUpdatingControls(true); }
    ~c_update_controls_lock() { _this->setUpdatingControls(false); }
  };


  QSettingsWidget(QWidget * parent = nullptr);

  void set_mutex(std::mutex * mtx);
  std::mutex* mutex();

  virtual void setUpdatingControls(bool v);
  virtual bool updatingControls();

  void loadSettings(const QString & prefix = "");
  void loadSettings(const QSettings & settings, const QString & prefix = "");
  void saveSettings(const QString & prefix = "");
  void saveSettings(QSettings & settings, const QString & prefix = "");

public Q_SLOTS:
  void updateControls();
  virtual void onload(const QSettings & settings, const QString & prefix = "");
  virtual void onsave(QSettings & settings, const QString & prefix = "");

Q_SIGNALS:
  void parameterChanged();
  void enablecontrols();
  void populatecontrols();
  void groupExpanded();
  void groupCollapsed();

protected:
  virtual void lock();
  virtual void unlock();

protected:
  QFormLayout *form = nullptr;

private:
  std::mutex *_mtx = nullptr;
  int _updatingControls = 0;
public:

  /////////////////////////////////////////////////////////////////////

  void addRow(const QString & label, QWidget * field)
  {
    form->addRow(label, field);
  }

  void addRow(QWidget * field)
  {
    form->addRow(field);
  }

  void insertRow(int row, QWidget * w)
  {
    form->insertRow(row, w);
  }

  void removeWidget(QWidget * w)
  {
    form->removeWidget(w);
  }

  int rowCount() const
  {
    return form->rowCount();
  }

  /////////////////////////////////////////////////////////////////////

  template<class T>
  QNumericBox * add_numeric_box(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn = nullptr,
      const std::function<bool(T*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QNumericBox * ctl = new QNumericBox(this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if ( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QNumericBox::textChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              T v;
              if ( fromString(ctl->text(), &v) ) {
                c_mutex_lock lock(this);
                setfn(v);
              }
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            T v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setValue(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  template<class T>
  QNumericBox * add_numeric_box(const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn = nullptr,
      const std::function<bool(T*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_numeric_box<T>(this->form, name, tooltip, setfn, getfn, enablefn);
  }


  /////////////////////////////////////////////////////////////////////

  QLineEditBox* add_textbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QLineEditBox *ctl = new QLineEditBox();
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QLineEditBox::textChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->text());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            QString v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setText(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QLineEditBox* add_textbox(const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_textbox(this->form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = nullptr,
      const std::function<bool(bool*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QCheckBox *ctl = new QCheckBox(this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QCheckBox::stateChanged,
          [this, setfn](int state) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(state==Qt::Checked);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            bool v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setChecked(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QCheckBox * add_checkbox(const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = nullptr,
      const std::function<bool(bool*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_checkbox(this->form, name, tooltip, setfn, getfn, enablefn);
  }


  /////////////////////////////////////////////////////////////////////


  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = nullptr,
      const std::function<bool(bool*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QCheckBox *ctl = new QCheckBox(name, this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QCheckBox::stateChanged,
          [this, setfn](int state) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(state==Qt::Checked);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            bool v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setChecked(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }


  QCheckBox * add_named_checkbox(const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = nullptr,
      const std::function<bool(bool*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_checkbox(this->form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(EnumType)> & setfn = nullptr,
      const std::function<bool(EnumType*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    typedef QEnumComboBox<EnumType> CombomoxType;

    CombomoxType * ctl = new CombomoxType(this);
    QSignalBlocker block(ctl);
    ctl->setFocusPolicy(Qt::StrongFocus);
    ctl->setWhatsThis(tooltip);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->currentItem());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            EnumType v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setCurrentItem(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }


  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(const QString & name, const QString & tooltip,
      const std::function<void(EnumType)> & setfn = nullptr,
      const std::function<bool(EnumType*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_enum_combobox<EnumType>(this->form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  QEnumComboBoxBase * add_enum_combobox_base(QFormLayout * form, const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    typedef QEnumComboBoxBase CombomoxType;

    CombomoxType * ctl = new CombomoxType(members, this);
    QSignalBlocker block(ctl);
    ctl->setFocusPolicy(Qt::StrongFocus);
    ctl->setWhatsThis(tooltip);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
          [this, ctl, setfn](int index) {
            if ( index >= 0 && !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->itemData(index).toInt());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [this, ctl, getfn]() {
            int v;
            if ( getfn(&v) && (v = ctl->findData(v)) >= 0 ) {
              QSignalBlocker block(ctl);
              ctl->setCurrentIndex(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QEnumComboBoxBase* add_enum_combobox_base(const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_enum_combobox_base(this->form, name, tooltip, members, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class EnumType>
  QFlagsEditBox<EnumType> * add_flags_editbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QFlagsEditBox<EnumType> *ctl = new QFlagsEditBox<EnumType>(this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QFlagsEditBoxBase::flagsChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->flags());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            int v = 0;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setFlags(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  template<class EnumType>
  QFlagsEditBox<EnumType> * add_flags_editbox(const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_flags_editbox<EnumType>(this->form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  QFlagsEditBoxBase * add_flags_editbox_base(QFormLayout * form, const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QFlagsEditBoxBase *ctl = new QFlagsEditBoxBase(members, this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QFlagsEditBoxBase::flagsChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->flags());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            int v = 0;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setFlags(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
        [conn](QObject * obj) {
          obj->disconnect(conn);
        });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QFlagsEditBoxBase * add_flags_editbox_base(const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_flags_editbox_base(this->form, name, tooltip, members, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class ComboBoxType = QComboBox>
  ComboBoxType* add_combobox(QFormLayout * form, const QString & name, const QString & tooltip, bool editable,
      const std::function<void(int, ComboBoxType*)> & setfn = nullptr,
      const std::function<bool(int*, ComboBoxType*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    ComboBoxType *ctl = new ComboBoxType();
    QSignalBlocker block(ctl);
    ctl->setFocusPolicy(Qt::StrongFocus);
    ctl->setToolTip(tooltip);
    ctl->setEditable(editable);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          [this, ctl, setfn](int v) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(v, ctl);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            int v = -1;
            if ( getfn(&v, ctl) ) {
              QSignalBlocker block(ctl);
              ctl->setCurrentIndex(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }


  template<class ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const QString & tooltip, bool editable,
      const std::function<void(int, ComboBoxType*)> & setfn = nullptr,
      const std::function<bool(int*, ComboBoxType*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_combobox<ComboBoxType>(this->form, name, tooltip, editable, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////
  QSpinBox* add_spinbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QSpinBox *ctl = new QSpinBox();
    QSignalBlocker block(ctl);
    ctl->setKeyboardTracking(false);
    ctl->setFocusPolicy(Qt::StrongFocus);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          [this, ctl, setfn](int v) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            int v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setValue(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QSpinBox* add_spinbox(const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn = nullptr,
      const std::function<bool(int*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_spinbox(this->form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////
  QDoubleSpinBox * add_double_spinbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(double)> & setfn = nullptr,
      const std::function<bool(double*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QDoubleSpinBox *ctl = new QDoubleSpinBox();
    QSignalBlocker block(ctl);
    ctl->setKeyboardTracking(false);
    ctl->setFocusPolicy(Qt::StrongFocus);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
          [this, ctl, setfn](double v) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            double v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setValue(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QDoubleSpinBox* add_double_spinbox(const QString & name, const QString & tooltip,
      const std::function<void(double)> & setfn = nullptr,
      const std::function<bool(double*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_double_spinbox(this->form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////
  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn = nullptr,
      const std::function<bool(T*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    typename QSliderSpinBox<T>::Type *ctl = new typename QSliderSpinBox<T>::Type();
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QSliderSpinBox<T>::Type::valueChanged,
          [this, ctl, setfn](T v) {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
    QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            T v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setValue(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn = nullptr,
      const std::function<bool(T*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_sliderspinbox<T>(this->form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  QBrowsePathCombo * add_browse_for_path(QFormLayout * form, const QString & name, const QString & label,
      QFileDialog::AcceptMode acceptMode, QFileDialog::FileMode fileMode,
      const std::function<void(const QString &)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QBrowsePathCombo *ctl = new QBrowsePathCombo(label, acceptMode, fileMode);
    QSignalBlocker block(ctl);
    if ( name.isEmpty() ) {
      form->addRow(ctl);
    }
    else {
      form->addRow(name, ctl);
    }

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QBrowsePathCombo::pathChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->currentPath());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            QString v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setCurrentPath(v, false);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QBrowsePathCombo * add_browse_for_path(const QString & name, const QString & label,
      QFileDialog::AcceptMode acceptMode, QFileDialog::FileMode fileMode,
      const std::function<void(const QString &)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_browse_for_path(form, name, label, acceptMode, fileMode, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  QInputMathExpressionWidget* add_math_expression(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool(QString*)> & helpfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QInputMathExpressionWidget * ctl = new QInputMathExpressionWidget(this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    if( name.isEmpty() ) {
      form->addRow(ctl);
    }
    else {
      form->addRow(name, ctl);
    }

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QInputMathExpressionWidget::apply,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->text());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            QString v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setText(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( helpfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, helpfn]() {
            QString v;
            if ( helpfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setHelpString(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }


    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }
    return ctl;
  }

  QInputMathExpressionWidget * add_math_expression(const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool(QString*)> & helpfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_math_expression(form, name, tooltip, setfn, getfn, helpfn, enablefn);
  }


  /////////////////////////////////////////////////////////////////////
  QDataAnnotationSelectorCtrl * add_data_annotation_ctl(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int cmap, int lb)> & setfn = nullptr,
      const std::function<bool(int cmap, int * lb)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QDataAnnotationSelectorCtrl * ctl = new QDataAnnotationSelectorCtrl(this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
    }
    if( getfn ) {
    }
    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QDataAnnotationSelectorCtrl * add_data_annotation_ctl(const QString & name, const QString & tooltip,
      const std::function<void(int cmap, int lb)> & setfn = nullptr,
      const std::function<bool(int cmap, int * lb)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_data_annotation_ctl(form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  QFFmpegOptionsControl* add_ffmpeg_options_control(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QFFmpegOptionsControl * ctl = new QFFmpegOptionsControl(this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QFFmpegOptionsControl::textChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->text());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [ctl, getfn]() {
            QString v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setText(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QFFmpegOptionsControl * add_ffmpeg_options_control(const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = nullptr,
      const std::function<bool(QString*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_ffmpeg_options_control(form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  QColorPickerButton* add_color_picker_button(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(const QColor&)> & setfn = nullptr,
      const std::function<bool(QColor*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    QColorPickerButton * ctl = new QColorPickerButton(this);
    QSignalBlocker block(ctl);
    ctl->setToolTip(tooltip);
    form->addRow(name, ctl);

    if( setfn ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl, &QColorPickerButton::colorSelected,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              c_mutex_lock lock(this);
              setfn(ctl->color());
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( getfn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
          [this, ctl, getfn, enablefn]() {
            QColor v;
            if ( getfn(&v) ) {
              QSignalBlocker block(ctl);
              ctl->setColor(v);
            }
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    if( enablefn ) {
      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::enablecontrols,
          [ctl, enablefn]() {
            ctl->setEnabled(enablefn());
          });
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QColorPickerButton * add_color_picker_button(const QString & name, const QString & tooltip,
      const std::function<void(const QColor&)> & setfn = nullptr,
      const std::function<bool(QColor*)> & getfn = nullptr,
      const std::function<bool()> & enablefn = nullptr)
  {
    return add_color_picker_button(form, name, tooltip, setfn, getfn, enablefn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class T, class S, class G>
  typename std::enable_if<std::is_arithmetic<T>::value && !std::is_same<T, bool>::value,
  QNumericBox*>::type add_ctl(const QString & name, const QString & tooltip, const S & setfn, const G & getfn)
  {
    return add_numeric_box<T>(name, tooltip, setfn, getfn);
  }

  template<class T, class S, class G>
  typename std::enable_if<std::is_same<T, bool>::value,
  QCheckBox*>::type add_ctl(const QString & name, const QString & tooltip, const S & setfn, const G & getfn)
  {
    return add_checkbox(name, tooltip, setfn, getfn);
  }

  template<class T, class S, class G>
  typename std::enable_if<std::is_enum<T>::value,
  QEnumComboBox<T>*>::type add_ctl(const QString & name, const QString & tooltip, const S & setfn, const G & getfn)
  {
    return add_enum_combobox<T>(name, tooltip, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QExpandableGroupBox* add_expandable_groupbox(QFormLayout * form, const QString & title, QWidget * ctl = nullptr,
      int stretch = 0, Qt::Alignment alignment = Qt::Alignment())
  {
    QExpandableGroupBox *gbox = new QExpandableGroupBox(title, ctl, stretch, alignment);
    form->addRow(gbox);
    return gbox;
  }

  QExpandableGroupBox * add_expandable_groupbox(const QString & title, QWidget * ctl = nullptr,
      int stretch = 0, Qt::Alignment alignment = Qt::Alignment())
  {
    return add_expandable_groupbox(this->form, title, ctl, stretch, alignment);
  }

  /////////////////////////////////////////////////////////////////////
  QToolButton * add_tool_button(QFormLayout * form, const QString & name, const QIcon & icon,
      const std::function<void(bool)> & onclick = std::function<void(bool)>())
  {
    QToolButton * ctl = new QToolButton(this);
    QSignalBlocker block(ctl);
    ctl->setToolButtonStyle(name.isEmpty() ? Qt::ToolButtonStyle::ToolButtonIconOnly : Qt::ToolButtonStyle::ToolButtonTextBesideIcon);
    ctl->setIconSize(QSize(16,16));
    ctl->setText(name);
    ctl->setIcon(icon);
    form->addRow(ctl);

    if( onclick ) {
      QMetaObject::Connection conn =
        QObject::connect(ctl,
          &QToolButton::clicked,
          onclick);
      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QToolButton * add_tool_button(const QString & name, const QIcon & icon,
      const std::function<void(bool)> & onclick = std::function<void(bool)>())
  {
    return add_tool_button(form, name, icon, onclick);
  }

  QToolButton * add_tool_button(const QString & name,
      const std::function<void(bool)> & onclick = std::function<void(bool)>())
  {
    return add_tool_button(form, name, QIcon(), onclick);
  }


  /////////////////////////////////////////////////////////////////////

  template<class WidgetType>
  WidgetType * add_widget(QFormLayout * form, const QString & name = QString())
  {
    WidgetType * ctl = new WidgetType(this);
    QSignalBlocker block(ctl);

    if ( !name.isEmpty() ) {
      form->addRow(name, ctl);
    }
    else {
      form->addRow(ctl);
    }

    return ctl;
  }

  template<class WidgetType>
  WidgetType * add_widget(const QString & name = QString())
  {
    return add_widget<WidgetType>(this->form, name);
  }

  template<class WidgetType, class _UpdateSignal, class _UpdateCallback,class _PopuLateCallback>
  WidgetType * add_widget2(const QString & name, const _UpdateSignal & s, const _UpdateCallback & ucb, const _PopuLateCallback & pcb)
  {
    WidgetType * ctl = new WidgetType(this);
    QSignalBlocker block(ctl);

    if ( !name.isEmpty() ) {
      form->addRow(name, ctl);
    }
    else {
      form->addRow(ctl);
    }

    QMetaObject::Connection cn1 = QObject::connect(ctl, s, ucb);
    QMetaObject::Connection cn2 = QObject::connect(this, &ThisClass::populatecontrols, pcb);
    QObject::connect(ctl, &QObject::destroyed,
        [cn1, cn2](QObject * obj) {
          obj->disconnect(cn1);
          obj->disconnect(cn2);
    });

    return ctl;
  }

  /////////////////////////////////////////////////////////////////////
};


template<class StructType, class QSettingsWidgetBase = QSettingsWidget>
class QSettingsWidgetTemplate :
    public QSettingsWidgetBase
{
  // Q_OBJECT;
public:
  typedef QSettingsWidgetTemplate ThisClass;
  typedef QSettingsWidgetBase Base;
  typedef StructType OptsType;

  QSettingsWidgetTemplate(QWidget * parent = nullptr) :
    Base(parent)
  {
    QObject::connect(this, &ThisClass::enablecontrols,
        [this]() {
          this->setEnabled(_opts != nullptr);
        });
  }

  virtual void setOpts(OptsType * opts)
  {
    this->_opts = opts;
    Base::updateControls();
  }

  OptsType * opts() const
  {
    return _opts;
  }

protected:
  OptsType * _opts = nullptr;
};


class QSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QSettingsDialogBox(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags()) :
    Base(parent, flags)
  {
  }

  QSettingsDialogBox(const QString & title, QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags()) :
    Base(parent, flags)
  {
    setWindowTitle(title);
  }

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void parameterChanged();

protected:
  void showEvent(QShowEvent *event) override
  {
    Base::showEvent(event);
    Q_EMIT visibilityChanged(isVisible());
  }

  void hideEvent(QHideEvent *event) override
  {
    Base::hideEvent(event);
    Q_EMIT visibilityChanged(isVisible());
  }
};

template<class QSettingsWidgetType>
class QSettingsDialogBoxTemplate :
    public QSettingsDialogBox
{
public:
  typedef QSettingsDialogBoxTemplate ThisClass;
  typedef QSettingsDialogBox Base;
  typedef QSettingsWidgetType SettingsWidgetType;
  typedef typename QSettingsWidgetType::OptsType OptsType;

  QSettingsDialogBoxTemplate(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags()) :
      ThisClass("", parent, flags)
  {
  }

  QSettingsDialogBoxTemplate(const QString & title, QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags()) :
    Base(title, parent, flags)
  {
    _layout = new QVBoxLayout(this);
    _layout->addWidget(_settings = new QSettingsWidgetType(this));
    connect(_settings, &QSettingsWidgetType::parameterChanged,
        this, &ThisClass::parameterChanged);
  }

  SettingsWidgetType * settingsWidget() const
  {
    return _settings;
  }

  void setOpts(OptsType * opts)
  {
    _settings->setOpts(opts);
  }

  OptsType * opts() const
  {
    return _settings->opts();
  }

  void loadSettings(const QString & prefix = "")
  {
    _settings->loadSettings(prefix);
  }

  void loadSettings(const QSettings & settings, const QString & prefix = "")
  {
    _settings->loadSettings(settings, prefix);
  }

  void saveSettings(const QString & prefix = "")
  {
    _settings->saveSettings(prefix);
  }

  void saveSettings(QSettings & settings, const QString & prefix = "")
  {
    _settings->saveSettings(settings, prefix);
  }

protected:
  QVBoxLayout * _layout;
  QSettingsWidgetType * _settings = nullptr;
};

////////////////////////////
#endif /* __QSettingsWidget_h__ */
