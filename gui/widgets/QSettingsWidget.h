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
#include <mutex>

class QSettingsWidget
    : public QWidget
{
  Q_OBJECT;

public:
  typedef QWidget Base;

  class auto_lock {
    std::mutex * mtx_;
    public:
    auto_lock(QSettingsWidget * obj)
    {
      if ( (mtx_ = obj->mtx_) ) {
        mtx_->lock();
      }
    }
    ~auto_lock()
    {
      if ( mtx_ ) {
        mtx_->unlock();
      }
    }
  };


  QSettingsWidget(const QString & prefix,  QWidget  * parent = Q_NULLPTR);

  void setSettingsPrefix(const QString & v);
  const QString & settingsPrefix() const;

  void set_mutex(std::mutex * mtx);
  std::mutex * mutex();

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
  template<class _Calable>
  QCheckBox * add_checkbox(QFormLayout * form, const char * name, const _Calable & slot)
  {
    QCheckBox * ctl = new QCheckBox(name);
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
  QCheckBox * add_checkbox(const char * name, const _Calable & slot) {
    return add_checkbox(this->form, name, slot);
  }

protected:
  QString PREFIX;
  QFormLayout * form = Q_NULLPTR;
private:
  std::mutex * mtx_ = Q_NULLPTR;
  bool updatingControls_ = false;
};




#endif /* __QSettingsWidget_h__ */
