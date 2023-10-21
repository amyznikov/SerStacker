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
#include <type_traits>
#include <functional>
#include <mutex>


class QSettingsWidget :
    public QFrame
{
  Q_OBJECT;

public:
  typedef QSettingsWidget ThisClass;
  typedef QFrame Base;

  struct c_mutex_lock
  {
    QSettingsWidget * this_;

    c_mutex_lock(QSettingsWidget * _this) : this_(_this) {
      this_->lock();
    }

    ~c_mutex_lock() {
      this_->unlock();
    }
  };

  struct c_update_controls_lock {
    ThisClass * _this;
    c_update_controls_lock(ThisClass * obj) : _this(obj) {
      _this->setUpdatingControls(true);
    }
    ~c_update_controls_lock() {
      _this->setUpdatingControls(false);
    }
  };


  QSettingsWidget(const QString & prefix, QWidget * parent = nullptr);

  void setSettingsPrefix(const QString & v);
  const QString& settingsPrefix() const;

  void set_mutex(std::mutex * mtx);
  std::mutex* mutex();

  void loadSettings(QSettings & settings);

  virtual void setUpdatingControls(bool v);
  virtual bool updatingControls();

public Q_SLOTS:
  void loadParameters();
  void updateControls();

Q_SIGNALS:
  void parameterChanged();
  void populatecontrols();

protected:
  virtual void onload(QSettings & settings);
  virtual void onupdatecontrols();
  virtual void lock();
  virtual void unlock();

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
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    QNumericBox * ctl = new QNumericBox(this);
    if ( !tooltip.isEmpty() ) {
      ctl->setToolTip(tooltip);
    }
    form->addRow(name, ctl);


    if ( setfn ) {

      QMetaObject::Connection conn =
          QObject::connect(ctl, &QNumericBox::textChanged,
              [this, ctl, setfn]() {
                if ( !updatingControls() && setfn ) {
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
    return ctl;
  }

  template<class T>
  QNumericBox * add_numeric_box(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn, const std::function<bool(T*)> & getfn)
  {
    QNumericBox * ctl =
        add_numeric_box(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
        QObject::connect(this, &ThisClass::populatecontrols,
            [ctl, getfn]() {
              T v;
              if ( getfn(&v) ) {
                ctl->setValue(v);
              }
            });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });

    }

    return ctl;
  }

//  template<class T>
//  QNumberEditBox * add_numeric_box(const QString & name, const std::function<void(T)> & setfn = std::function<void(T)>() )
//  {
//    return add_numeric_box<T>(this->form, name, setfn);
//  }

  template<class T>
  QNumericBox * add_numeric_box(const QString & name, const QString & tooltip, const std::function<void(T)> & setfn = std::function<void(T)>() )
  {
    return add_numeric_box<T>(this->form, name, tooltip, setfn);
  }

  template<class T>
  QNumericBox * add_numeric_box(const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn,
      const std::function<bool(T*)> & getfn)
  {
    return add_numeric_box<T>(this->form, name, tooltip, setfn, getfn);
  }


  /////////////////////////////////////////////////////////////////////

  QLineEditBox* add_textbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = std::function<void(const QString&)>(),
      const std::function<bool(QString*)> & getfn = std::function<bool(QString*)>())
  {
    QLineEditBox *ctl =
        new QLineEditBox();

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
                  ctl->setText(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }


    return ctl;
  }

  QLineEditBox* add_textbox(const QString & name, const QString & tooltip,
      const std::function<void(const QString&)> & setfn = std::function<void(const QString&)>(),
      const std::function<bool(QString*)> & getfn = std::function<bool(QString*)>())
  {
    return add_textbox(form, name, tooltip, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = std::function<void(bool)>())
  {
    QCheckBox *ctl = new QCheckBox(this);
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

    return ctl;
  }

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn , const std::function<bool(bool*)> & getfn)
  {
    QCheckBox *ctl = add_checkbox(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                bool v;
                if ( getfn(&v) ) {
                  ctl->setChecked(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QCheckBox * add_checkbox(const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = std::function<void(bool)>() )
  {
    return add_checkbox(this->form, name, tooltip, setfn);
  }

  QCheckBox * add_checkbox(const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn,
      const std::function<bool(bool*)> & getfn)
  {
    return add_checkbox(this->form, name, tooltip, setfn, getfn);
  }


  /////////////////////////////////////////////////////////////////////


  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = std::function<void(bool)>())
  {
    QCheckBox *ctl = new QCheckBox(name, this);
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

    return ctl;
  }

  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn , const std::function<bool(bool*)> & getfn)
  {
    QCheckBox *ctl =
        add_checkbox(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                bool v;
                if ( getfn(&v) ) {
                  ctl->setChecked(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QCheckBox * add_named_checkbox(const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn = std::function<void(bool)>() )
  {
    return add_checkbox(this->form, name, tooltip, setfn);
  }

  QCheckBox * add_named_checkbox(const QString & name, const QString & tooltip,
      const std::function<void(bool)> & setfn,
      const std::function<bool(bool*)> & getfn)
  {
    return add_checkbox(this->form, name, tooltip, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(EnumType)> & setfn = std::function<void(EnumType)>())
  {
    typedef QEnumComboBox<EnumType>
      CombomoxType;

    CombomoxType * ctl =
        new CombomoxType(this);

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

    return ctl;
  }

  template<class EnumType>
  QEnumComboBox<EnumType>* add_enum_combobox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(EnumType)> & setfn, const std::function<bool(EnumType*)> & getfn)
  {
    QEnumComboBox<EnumType> *ctl =
        add_enum_combobox(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                EnumType v;
                if ( getfn(&v) ) {
                  ctl->setCurrentItem(v);
                }
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
      const std::function<void(EnumType)> & setfn = std::function<void(EnumType)>())
  {
    return add_enum_combobox<EnumType>(this->form, name, tooltip, setfn);
  }

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(const QString & name, const QString & tooltip,
      const std::function<void(EnumType)> & setfn, const std::function<bool(EnumType*)> & getfn)
  {
    return add_enum_combobox<EnumType>(this->form, name, tooltip, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QEnumComboBoxBase * add_enum_combobox_base(QFormLayout * form, const QString & name, const QString & tooltip,
      const c_enum_member * members, const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    typedef QEnumComboBoxBase
      CombomoxType;

    CombomoxType * ctl =
        new CombomoxType(members, this);

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

    return ctl;
  }

  QEnumComboBoxBase * add_enum_combobox_base(QFormLayout * form, const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    QEnumComboBoxBase *ctl =
        add_enum_combobox_base(form, name, tooltip, members, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v;
                if ( getfn(&v) && (v = ctl->findData(v)) >= 0 ) {
                  ctl->setCurrentIndex(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QEnumComboBoxBase * add_enum_combobox_base(const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    return add_enum_combobox_base(this->form, name, tooltip, members, setfn);
  }

  QEnumComboBoxBase* add_enum_combobox_base(const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    return add_enum_combobox_base(this->form, name, tooltip, members, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class EnumType>
  QFlagsEditBox<EnumType> * add_flags_editbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    QFlagsEditBox<EnumType> *ctl =
        new QFlagsEditBox<EnumType>(this);

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

    return ctl;
  }

  template<class EnumType>
  QFlagsEditBox<EnumType>* add_flags_editbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    QFlagsEditBox<EnumType> *ctl =
        add_flags_editbox<EnumType>(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v = 0;
                if ( getfn(&v) ) {
                  ctl->setFlags(v);
                }
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
      const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    return add_flags_editbox<EnumType>(this->form, name, tooltip, setfn);
  }

  template<class EnumType>
  QFlagsEditBox<EnumType> * add_flags_editbox(const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    return add_flags_editbox<EnumType>(this->form, name, tooltip, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////


  QFlagsEditBoxBase * add_flags_editbox_base(QFormLayout * form, const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    QFlagsEditBoxBase *ctl =
        new QFlagsEditBoxBase(members, this);

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

    return ctl;
  }

  QFlagsEditBoxBase * add_flags_editbox_base(QFormLayout * form, const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    QFlagsEditBoxBase *ctl =
        add_flags_editbox_base(form, name, tooltip, members, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v = 0;
                if ( getfn(&v) ) {
                  ctl->setFlags(v);
                }
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
      const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    return add_flags_editbox_base(this->form, name, tooltip, members, setfn);
  }

  QFlagsEditBoxBase * add_flags_editbox_base(const QString & name, const QString & tooltip,
      const c_enum_member * members,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    return add_flags_editbox_base(this->form, name, tooltip, members, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class ComboBoxType = QComboBox>
  ComboBoxType* add_combobox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int, ComboBoxType*)> & setfn = std::function<void(int, ComboBoxType*)>())
  {
    ComboBoxType *ctl =
        new ComboBoxType();

    ctl->setFocusPolicy(Qt::StrongFocus);
    ctl->setToolTip(tooltip);

    form->addRow(name, ctl);

    if( setfn ) {

      QMetaObject::Connection conn =
          QObject::connect(ctl,
              static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
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

    return ctl;
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType* add_combobox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int, ComboBoxType*)> & setfn,
      const std::function<bool(int*, ComboBoxType*)> & getfn)
  {
    ComboBoxType *ctl =
        add_combobox(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v = -1;
                if ( getfn(&v, ctl) ) {
                  ctl->setCurrentIndex(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  template<typename ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const QString & tooltip,
      const std::function<void(int, ComboBoxType*)> & setfn = std::function<void(int, ComboBoxType*)>())
  {
    return add_combobox<ComboBoxType>(this->form, name, tooltip, setfn);
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const QString & tooltip,
      const std::function<void(int, ComboBoxType*)> & setfn,
      const std::function<bool(int*, ComboBoxType*)> & getfn)
  {
    return add_combobox<ComboBoxType>(this->form, name, tooltip, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////
  QSpinBox* add_spinbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    QSpinBox *ctl = new QSpinBox();

    ctl->setKeyboardTracking(false);
    ctl->setFocusPolicy(Qt::StrongFocus);
    ctl->setToolTip(tooltip);

    form->addRow(name, ctl);

    if( setfn ) {

      QMetaObject::Connection conn =
          QObject::connect(ctl,
              static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
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

    return ctl;
  }

  QSpinBox* add_spinbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    QSpinBox *ctl =
        add_spinbox(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v;
                if ( getfn(&v) ) {
                  ctl->setValue(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QSpinBox* add_spinbox(const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    return add_spinbox(this->form, name, tooltip, setfn);
  }

  QSpinBox* add_spinbox(const QString & name, const QString & tooltip,
      const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    return add_spinbox(this->form, name, tooltip, setfn, getfn);
  }


  /////////////////////////////////////////////////////////////////////
  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    typename QSliderSpinBox<T>::Type *ctl =
        new typename QSliderSpinBox<T>::Type();

    ctl->setToolTip(tooltip);

    form->addRow(name, ctl);

    if( setfn ) {

      QMetaObject::Connection conn =
          QObject::connect(ctl,
              &QSliderSpinBox<T>::Type::valueChanged,
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

    return ctl;
  }

  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(QFormLayout * form, const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn, const std::function<bool(T*)> & getfn)
  {
    typename QSliderSpinBox<T>::Type *ctl =
        add_sliderspinbox<T>(form, name, tooltip, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                T v;
                if ( getfn(&v) ) {
                  ctl->setValue(v);
                }
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
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    return add_sliderspinbox<T>(this->form, name, tooltip, setfn);
  }

  template<class T>
  typename QSliderSpinBox<T>::Type * add_sliderspinbox(const QString & name, const QString & tooltip,
      const std::function<void(T)> & setfn, const std::function<bool(T*)> & getfn)
  {
    return add_sliderspinbox<T>(this->form, name, tooltip, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QBrowsePathCombo * add_browse_for_path(QFormLayout * form, const QString & name, const QString & label,
      QFileDialog::AcceptMode acceptMode,
      QFileDialog::FileMode fileMode,
      const std::function<void(const QString &)> & setfn = std::function<void(const QString &)>())
  {
    QBrowsePathCombo *ctl =
        new QBrowsePathCombo(label, acceptMode, fileMode);

    if ( name.isEmpty() ) {
      form->addRow(ctl);
    }
    else {
      form->addRow(name, ctl);
    }

    if( setfn ) {

      QMetaObject::Connection conn =
          QObject::connect(ctl,
              &QBrowsePathCombo::pathChanged,
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

    return ctl;
  }

  QBrowsePathCombo * add_browse_for_path(QFormLayout * form, const QString & name, const QString & label,
      QFileDialog::AcceptMode acceptMode,
      QFileDialog::FileMode fileMode,
      const std::function<void(const QString &)> & setfn,
      const std::function<bool(QString*)> & getfn)
  {
    QBrowsePathCombo *ctl =
        add_browse_for_path(form, name, label,
            acceptMode, fileMode, setfn);

    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                QString v;
                if ( getfn(&v) ) {
                  ctl->setCurrentPath(v, false);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn](QObject * obj) {
            obj->disconnect(conn);
          });
    }

    return ctl;
  }

  QBrowsePathCombo * add_browse_for_path(const QString & name, const QString & label,
      QFileDialog::AcceptMode acceptMode,
      QFileDialog::FileMode fileMode,
      const std::function<void(const QString &)> & setfn)

  {
    return add_browse_for_path(form, name, label, acceptMode, fileMode, setfn);
  }

  QBrowsePathCombo * add_browse_for_path(const QString & name, const QString & label,
      QFileDialog::AcceptMode acceptMode,
      QFileDialog::FileMode fileMode,
      const std::function<void(const QString &)> & setfn,
      const std::function<bool(QString*)> & getfn)
  {
    return add_browse_for_path(form, name, label, acceptMode, fileMode, setfn, getfn);
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

  /////////////////////////////////////////////////////////////////////

protected:
  QString PREFIX;
  QFormLayout *form = nullptr;

private:
  std::mutex *mtx_ = nullptr;
  int updatingControls_ = 0;
};


template<class OptionsType>
class QSettingsWidgetTemplate :
    public QSettingsWidget
{
public:
  typedef QSettingsWidgetTemplate ThisClass;
  typedef QSettingsWidget Base;
  typedef OptionsType c_options_type;


  QSettingsWidgetTemplate(QWidget * parent = nullptr) :
    ThisClass("", parent)
  {
  }

  QSettingsWidgetTemplate(const QString & prefix, QWidget * parent = nullptr) :
    Base(prefix, parent)
  {
  }

  void set_options(c_options_type * options)
  {
    this->options_ = options;
    updateControls();
  }

  c_options_type * options() const
  {
    return options_;
  }

protected:
  virtual void update_control_states()
  {
  }

  void onupdatecontrols() override
  {
    if ( !options_ ) {
      setEnabled(false);
    }
    else {
      Base::populatecontrols();
      update_control_states();
      setEnabled(true);
    }
  }

protected:
  c_options_type * options_ = nullptr;
};

#endif /* __QSettingsWidget_h__ */
