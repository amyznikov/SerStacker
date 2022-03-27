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
#include <functional>
#include <mutex>

class QSettingsWidget
    : public QWidget
{
  Q_OBJECT;

public:
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

protected:
  virtual void onload(QSettings & settings);
  virtual void onupdatecontrols();
  virtual bool updatingControls();
  virtual void setUpdatingControls(bool v);
  virtual void LOCK();
  virtual void UNLOCK();

protected:

  template<class T>
  QNumberEditBox* add_numeric_box(QFormLayout * form, const QString & name,
      const std::function<void(T)> & slot = std::function<void(T)>())
  {
    QNumberEditBox *ctl = new QNumberEditBox(this);
    form->addRow(name, ctl);
    QObject::connect(ctl, &QNumberEditBox::textChanged,
        [this, ctl, slot]() {
          if ( !updatingControls() && slot ) {
            T v;
            if ( fromString(ctl->text(), &v) ) {
              LOCK();
              slot(v);
              UNLOCK();
            }
          }
        });
    return ctl;
  }

  template<class T>
  QNumberEditBox* add_numeric_box(const QString & name, const std::function<void(T)> & slot = std::function<void(T)>() )
  {
    return add_numeric_box<T>(this->form, name, slot);
  }

  QLineEditBox* add_textbox(QFormLayout * form, const QString & name,
      const std::function<void(const QString&)> & slot = std::function<void(const QString&)>())
  {
    QLineEditBox *ctl = new QLineEditBox();
    form->addRow(name, ctl);
    QObject::connect(ctl, &QLineEditBox::textChanged,
        [this, ctl, slot]() {
          if ( !updatingControls() && slot ) {
            LOCK();
            slot(ctl->text());
            UNLOCK();
          }
        });

    return ctl;
  }

  QLineEditBox* add_textbox(const QString & name,
      const std::function<void(const QString&)> & slot = std::function<void(const QString&)>())
  {
    return add_textbox(this->form, name, slot);
  }

  QCheckBox* add_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(int)> & slot = std::function<void(int)>())
  {
    QCheckBox *ctl = new QCheckBox(this);
    form->addRow(name, ctl);
    QObject::connect(ctl, &QCheckBox::stateChanged,
        [this, slot](int state) {
          if ( !updatingControls() && slot ) {
              LOCK();
              slot(state);
              UNLOCK();
          }
        });
    return ctl;
  }

  QCheckBox* add_named_checkbox(QFormLayout * form, const QString & name,
      const std::function<void(int)> & slot = std::function<void(int)>())
  {
    QCheckBox *ctl = new QCheckBox(name, this);
    form->addRow(ctl);
    QObject::connect(ctl, &QCheckBox::stateChanged,
        [this, slot](int state) {
          if ( !updatingControls() && slot ) {
              LOCK();
              slot(state);
              UNLOCK();
          }
        });
    return ctl;
  }

  QCheckBox * add_checkbox(const QString & name, const std::function<void(int)> & slot = std::function<void(int)>() )
  {
    return add_checkbox(this->form, name, slot);
  }

  QCheckBox * add_named_checkbox(const QString & name, const std::function<void(int)> & slot = std::function<void(int)>() )
  {
    return add_named_checkbox(this->form, name, slot);
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(QFormLayout * form, const QString & name,
      const std::function<void(int)> & slot = std::function<void(int)>())
  {
    ComboBoxType * ctl = new ComboBoxType(this);
    form->addRow(name, ctl);
    QObject::connect(ctl,
        static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        [this, slot](int currentIndex) {
          if ( !updatingControls() && slot ) {
            LOCK();
            slot(currentIndex);
            UNLOCK();
          }
        });
    return ctl;
  }

  template<class ComboBoxType = QComboBox>
  ComboBoxType * add_combobox(const QString & name, const std::function<void(int)> & slot = std::function<void(int)>())
  {
    return add_combobox<ComboBoxType>(this->form, name, slot);
  }

  template<class EnumType, class _Calable>
  QEnumComboBox<EnumType> * add_enum_combobox(QFormLayout * form, const QString & name, const _Calable & slot)
  {
    typedef QEnumComboBox<EnumType> CombomoxType;
    CombomoxType * ctl = new CombomoxType(this);
    form->addRow(name, ctl);
    QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
        [this, ctl, slot](int) {
          if ( !updatingControls() ) {
            LOCK();
            slot(ctl->currentItem());
            UNLOCK();
          }
        });
    return ctl;
  }

  template<class EnumType, class _Calable>
  QEnumComboBox<EnumType> * add_enum_combobox(const QString & name, const _Calable & slot)
  {
    return add_enum_combobox<EnumType>(this->form, name, slot);
  }

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

protected:
  QString PREFIX;
  QFormLayout *form = Q_NULLPTR;
  private:
  std::mutex *mtx_ = Q_NULLPTR;
  bool updatingControls_ = false;
};

#endif /* __QSettingsWidget_h__ */
