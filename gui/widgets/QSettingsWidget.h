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
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/QExpandableGroupBox.h>
#include <type_traits>
#include <functional>
#include <mutex>


class QSettingsWidget
    : public QWidget
{
  Q_OBJECT;

public:
  typedef QSettingsWidget ThisClass;
  typedef QWidget Base;

  class auto_lock
  {
    std::mutex *mtx_;
    public:
    auto_lock(QSettingsWidget * obj)
    {
      if( (mtx_ = obj->mtx_) ) {
        mtx_->lock();
      }
    }
    ~auto_lock()
    {
      if( mtx_ ) {
        mtx_->unlock();
      }
    }
  };

  QSettingsWidget(const QString & prefix, QWidget * parent = Q_NULLPTR);

  void setSettingsPrefix(const QString & v);
  const QString& settingsPrefix() const;

  void set_mutex(std::mutex * mtx);
  std::mutex* mutex();

  void loadSettings(QSettings & settings);

public slots:
  void updateControls();
  void loadParameters();

signals:
  void parameterChanged();
  void populatecontrols();

protected:
  virtual void onload(QSettings & settings);
  virtual void onupdatecontrols();
  virtual bool updatingControls();
  virtual void setUpdatingControls(bool v);
  virtual void LOCK();
  virtual void UNLOCK();

public:

  /////////////////////////////////////////////////////////////////////

  template<class T>
  QNumberEditBox * add_numeric_box(QFormLayout * form, const QString & name,
      const std::function<void(T)> & setfn = std::function<void(T)>())
  {
    QNumberEditBox * ctl = new QNumberEditBox(this);
    form->addRow(name, ctl);

    if ( setfn ) {
      QObject::connect(ctl, &QNumberEditBox::textChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() && setfn ) {
              T v;
              if ( fromString(ctl->text(), &v) ) {
                LOCK();
                setfn(v);
                UNLOCK();
              }
            }
          });
    }
    return ctl;
  }

  template<class T>
  QNumberEditBox * add_numeric_box(QFormLayout * form, const QString & name,
      const std::function<void(T)> & setfn, const std::function<bool(T*)> & getfn)
  {
    QNumberEditBox * ctl = add_numeric_box(form, name, setfn);

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
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  template<class T>
  QNumberEditBox * add_numeric_box(const QString & name, const std::function<void(T)> & setfn = std::function<void(T)>() )
  {
    return add_numeric_box<T>(this->form, name, setfn);
  }

  template<class T>
  QNumberEditBox * add_numeric_box(const QString & name,const std::function<void(T)> & setfn , const std::function<bool(T*)> & getfn)
  {
    return add_numeric_box<T>(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QLineEditBox* add_textbox(QFormLayout * form, const QString & name,
      const std::function<void(const QString&)> & setfn = std::function<void(const QString&)>())
  {
    QLineEditBox *ctl = new QLineEditBox();
    form->addRow(name, ctl);

    if ( setfn ) {
      QObject::connect(ctl, &QLineEditBox::textChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              LOCK();
              setfn(ctl->text());
              UNLOCK();
            }
          });
    }

    return ctl;
  }

  QLineEditBox* add_textbox(const QString & name,
      const std::function<void(const QString&)> & slot = std::function<void(const QString&)>())
  {
    return add_textbox(this->form, name, slot);
  }

  /////////////////////////////////////////////////////////////////////

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn = std::function<void(bool)>())
  {
    QCheckBox *ctl = new QCheckBox(this);
    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl, &QCheckBox::stateChanged,
          [this, setfn](int state) {
            if ( !updatingControls() ) {
              LOCK();
              setfn(state==Qt::Checked);
              UNLOCK();
            }
          });
    }

    return ctl;
  }

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn , const std::function<bool(bool*)> & getfn)
  {
    QCheckBox *ctl = add_checkbox(form, name, setfn);

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
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  QCheckBox * add_checkbox(const QString & name, const std::function<void(bool)> & setfn = std::function<void(bool)>() )
  {
    return add_checkbox(this->form, name, setfn);
  }

  QCheckBox * add_checkbox(const QString & name, const std::function<void(bool)> & setfn, const std::function<bool(bool*)> & getfn)
  {
    return add_checkbox(this->form, name, setfn, getfn);
  }


  /////////////////////////////////////////////////////////////////////


  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn = std::function<void(bool)>())
  {
    QCheckBox *ctl = new QCheckBox(name, this);
    form->addRow(ctl);

    if( setfn ) {
      QObject::connect(ctl, &QCheckBox::stateChanged,
          [this, setfn](int state) {
            if ( !updatingControls() ) {
              LOCK();
              setfn(state==Qt::Checked);
              UNLOCK();
            }
          });
    }

    return ctl;
  }

  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(bool)> & setfn , const std::function<bool(bool*)> & getfn)
  {
    QCheckBox *ctl = add_checkbox(form, name, setfn);

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
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  QCheckBox * add_named_checkbox(const QString & name, const std::function<void(bool)> & setfn = std::function<void(bool)>() )
  {
    return add_checkbox(this->form, name, setfn);
  }

  QCheckBox * add_named_checkbox(const QString & name, const std::function<void(bool)> & setfn, const std::function<bool(bool*)> & getfn)
  {
    return add_checkbox(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(QFormLayout * form, const QString & name,
      const std::function<void(EnumType)> & setfn = std::function<void(EnumType)>())
  {
    typedef QEnumComboBox<EnumType> CombomoxType;
    CombomoxType * ctl = new CombomoxType(this);
    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
          [this, ctl, setfn]() {
            if ( !updatingControls() ) {
              LOCK();
              setfn(ctl->currentItem());
              UNLOCK();
            }
          });
    }

    return ctl;
  }

  template<class EnumType>
  QEnumComboBox<EnumType>* add_enum_combobox(QFormLayout * form, const QString & name,
      const std::function<void(EnumType)> & setfn, const std::function<bool(EnumType*)> & getfn)
  {
    QEnumComboBox<EnumType> *ctl = add_enum_combobox(form, name, setfn);

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
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(const QString & name,
      const std::function<void(EnumType)> & setfn = std::function<void(EnumType)>())
  {
    return add_enum_combobox<EnumType>(this->form, name, setfn);
  }

  template<class EnumType>
  QEnumComboBox<EnumType> * add_enum_combobox(const QString & name,
      const std::function<void(EnumType)> & setfn, const std::function<bool(EnumType*)> & getfn)
  {
    return add_enum_combobox<EnumType>(this->form, name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  template<class ComboBoxType = QComboBox>
  ComboBoxType* add_combobox(QFormLayout * form, const QString & name, const std::function<void(int)> & setfn =
      std::function<void(int)>())
  {
    ComboBoxType *ctl = new ComboBoxType();
    form->addRow(name, ctl);

    if( setfn ) {
      QObject::connect(ctl,
          static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          [this, ctl, setfn](int v) {
            if ( !updatingControls() ) {
              LOCK();
              setfn(v);
              UNLOCK();
            }
          });
    }

    return ctl;
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType* add_combobox(QFormLayout * form, const QString & name, const std::function<void(int)> & setfn,
      const std::function<bool(int*)> & getfn)
  {
    ComboBoxType *ctl = add_combobox(form, name, setfn);
    if( getfn ) {

      QMetaObject::Connection conn =
          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, getfn]() {
                int v;
                if ( getfn(&v) ) {
                  ctl->setCurrentIndex(v);
                }
              });

      QObject::connect(ctl, &QObject::destroyed,
          [conn]() {
            QObject::disconnect(conn);
          });
    }

    return ctl;
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const std::function<void(int)> & setfn = std::function<void(int)>())
  {
    return add_combobox<ComboBoxType>(this->form, name, setfn);
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const std::function<void(int)> & setfn, const std::function<bool(int*)> & getfn)
  {
    return add_combobox<ComboBoxType>(this->form, name, setfn, getfn);
  }


  /////////////////////////////////////////////////////////////////////

  template<class T, class S, class G>
  typename std::enable_if<std::is_arithmetic<T>::value && !std::is_same<T, bool>::value,
  QNumberEditBox*>::type add_ctl(const QString & name, const S & setfn, const G & getfn)
  {
    return add_numeric_box<T>(name, setfn, getfn);
  }

  template<class T, class S, class G>
  typename std::enable_if<std::is_same<T, bool>::value,
  QCheckBox*>::type add_ctl(const QString & name, const S & setfn, const G & getfn)
  {
    return add_checkbox(name, setfn, getfn);
  }

  template<class T, class S, class G>
  typename std::enable_if<std::is_enum<T>::value,
  QEnumComboBox<T>*>::type add_ctl(const QString & name, const S & setfn, const G & getfn)
  {
    return add_enum_combobox<T>(name, setfn, getfn);
  }

  /////////////////////////////////////////////////////////////////////

  QExpandableGroupBox* add_expandable_groupbox(QFormLayout * form, const QString & title, QWidget * ctl)
  {
    QExpandableGroupBox *gbox = new QExpandableGroupBox(title, ctl);
    form->addRow(gbox);
    return gbox;
  }

  QExpandableGroupBox * add_expandable_groupbox(const QString & title, QWidget * ctl)
  {
    return add_expandable_groupbox(this->form, title, ctl);
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
  QFormLayout *form = Q_NULLPTR;

private:
  std::mutex *mtx_ = Q_NULLPTR;
  bool updatingControls_ = false;
};

#endif /* __QSettingsWidget_h__ */
