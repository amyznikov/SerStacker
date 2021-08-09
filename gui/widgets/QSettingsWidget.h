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
#include <gui/widgets/QEnumComboBox.h>
#include <gui/widgets/QLineEditBox.h>
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

  template<class ValueType, class _Calable>
  QNumberEditBox* add_numeric_box(QFormLayout * form, const QString & name, const _Calable & slot)
  {
    QNumberEditBox *ctl = new QNumberEditBox(this);
    form->addRow(name, ctl);
    QObject::connect(ctl, &QNumberEditBox::textChanged,
        [this, ctl, slot]() {
          if ( !updatingControls() ) {
            ValueType v;
            if ( fromString(ctl->text(), &v) ) {
              LOCK();
              slot(v);
              UNLOCK();
            }
          }
        });
    return ctl;
  }

  template<class ValueType, class _Calable>
  QNumberEditBox* add_numeric_box(const QString & name, const _Calable & slot)
  {
    return add_numeric_box<ValueType>(this->form, name, slot);
  }


  template<class _Calable>
  QCheckBox* add_checkbox(QFormLayout * form, const QString & name, const _Calable & slot)
  {
    QCheckBox *ctl = new QCheckBox(name, this);
    form->addRow(ctl);
    QObject::connect(ctl, &QCheckBox::stateChanged,
        [this, slot](int state) {
          if ( !updatingControls() ) {
            LOCK();
            slot(state);
            UNLOCK();
          }
        });
    return ctl;
  }


  template<class _Calable>
  QCheckBox* add_checkbox(const QString & name, const _Calable & slot)
  {
    return add_checkbox(this->form, name, slot);
  }

  template<class _Calable>
  QComboBox* add_combobox(QFormLayout * form, const QString & name, const _Calable & slot)
  {
    QComboBox *ctl = new QComboBox(this);
    form->addRow(name, ctl);
    QObject::connect(ctl,
        static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        [this, slot](int currentIndex) {
          if ( !updatingControls() ) {
            LOCK();
            slot(currentIndex);
            UNLOCK();
          }
        });
    return ctl;
  }

  template<class _Calable>
  QComboBox* add_combobox(const QString & name, const _Calable & slot)
  {
    return add_combobox(this->form, name, slot);
  }

  template<class CombomoxType, class _Calable>
  CombomoxType* add_enum_combobox(QFormLayout * form, const QString & name, const _Calable & slot)
  {
    CombomoxType *ctl = new CombomoxType(this);
    form->addRow(name, ctl);
    QObject::connect(ctl, &CombomoxType::currentItemChanged,
        [this, ctl, slot](int currentIndex) {
          if ( !updatingControls() ) {
            LOCK();
            slot(ctl->currentItem());
            UNLOCK();
          }
        });
    return ctl;
  }

  template<class CombomoxType, class _Calable>
  CombomoxType * add_enum_combobox(const QString & name, const _Calable & slot)
  {
    return add_enum_combobox<CombomoxType>(this->form, name, slot);
  }

protected:
  QString PREFIX;
  QFormLayout *form = Q_NULLPTR;
  private:
  std::mutex *mtx_ = Q_NULLPTR;
  bool updatingControls_ = false;
};

#endif /* __QSettingsWidget_h__ */
