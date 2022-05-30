/*
 * QImageProcessorRoutineSettings.h
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageProcessorRoutineSettings_h__
#define __QImageProcessorRoutineSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/improc/c_image_processor.h>


class QImageProcessorRoutineSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageProcessorRoutineSettings ThisClass;
  typedef QSettingsWidget Base;

  static ThisClass * create(const c_image_processor_routine::ptr & routine,
      QWidget * parent = Q_NULLPTR);

  const c_image_processor_routine::ptr & routine() const;

signals:
  void moveUpRequested(ThisClass * _this);
  void moveDownRequested(ThisClass * _this);
  void addRoutineRequested(ThisClass * _this);
  void removeRoutineRequested(ThisClass * _this);

protected:
  QImageProcessorRoutineSettings(const c_image_processor_routine::ptr & routine, QWidget * parent = Q_NULLPTR);
  virtual void setup_controls();

protected:
  void onupdatecontrols() override;
  //void focusInEvent(QFocusEvent *event) override;


protected:
  c_image_processor_routine::ptr routine_;
  QWidget * header_ctl = Q_NULLPTR;
  QHBoxLayout * header_layout = Q_NULLPTR;
  QCheckBox * expand_ctl = Q_NULLPTR;
  QCheckBox * enable_ctl = Q_NULLPTR;
  QWidget * routine_ctl = Q_NULLPTR;
  QFormLayout * ctlform = Q_NULLPTR;

  QToolButton * move_up_ctl = Q_NULLPTR;
  QToolButton * move_down_ctl = Q_NULLPTR;
  QToolButton * menu_ctl = Q_NULLPTR;
};

//
//template<class ImageProcessorRoutine>
//class QImageProcessorRoutineSettings
//  : public QImageProcessorRoutineSettings
//{
//public:
//  typedef QImageProcessorRoutineSettings ThisClass;
//  typedef QImageProcessorRoutineSettings Base;
//  typedef ImageProcessorRoutine RoutineType;
//  typedef std::shared_ptr<RoutineType> RoutinePtr;
//
//
//  QImageProcessorRoutineSettings(const ClassFactory * classFactory, const RoutinePtr & routine, QWidget * parent = Q_NULLPTR) :
//      Base(classFactory, parent)
//  {
//    connect(enable_ctl, &QCheckBox::stateChanged,
//        [this](int state) {
//          if ( routine_ ) {
//            bool enabled = state == Qt::Checked;
//            if ( enabled != routine_->enabled() ) {
//              routine_->set_enabled(enabled);
//              emit parameterChanged();
//            }
//          }
//        });
//
//    set_routine(routine);
//  }
//
//  void set_routine(const RoutinePtr & routine) {
//    routine_ = routine;
//    updateControls();
//  }
//
//  const RoutinePtr & routine() const {
//    return routine_;
//  }
//
//  c_image_processor_routine::ptr current_routine() const override {
//    return routine_;
//  }
//
//protected:
//  void onupdatecontrols() override {
//    if ( routine_ ) {
//      enable_ctl->setChecked(routine_->enabled());
//      if ( routine_->enabled() ) {
//        emit populatecontrols();
//      }
//    }
//  }
//
//protected:
//
//  QCheckBox * add_checkbox(const char * name,
//      bool (RoutineType::*getfn)() const,
//      void (RoutineType::*setfn)(bool),
//      const std::function<void()> & onchange = std::function<void()>())
//  {
//    QCheckBox * ctl = new QCheckBox();
//    ctlform->addRow(name, ctl);
//    QObject::connect(ctl, &QCheckBox::stateChanged,
//        [this, ctl, getfn, setfn, onchange](int state) {
//          if ( routine_ && !updatingControls() ) {
//            const bool checked = state == Qt::Checked;
//            if ( (routine_.get()->*getfn)() != checked ) {
//              LOCK();
//              (routine_.get()->*setfn)(checked);
//              if ( onchange ) {
//                onchange();
//              }
//              UNLOCK();
//              if ( routine_->enabled() ) {
//                emit parameterChanged();
//              }
//            }
//          }
//        });
//
//    return ctl;
//  }
//
//  template<class PropType>
//  QNumberEditBox * add_numeric_box(const char * name,
//      PropType (RoutineType::*getfn)() const,
//      void (RoutineType::*setfn)(PropType))
//  {
//    QNumberEditBox * ctl = new QNumberEditBox();
//    ctlform->addRow(name, ctl);
//    QObject::connect(ctl, &QLineEditBox::textChanged,
//        [this, ctl, getfn, setfn]() {
//          if ( routine_ && !updatingControls() ) {
//            PropType value = (routine_.get()->*getfn)();
//            if ( fromString(ctl->text(), &value) && (routine_.get()->*getfn)() != value ) {
//              LOCK();
//              (routine_.get()->*setfn)(value);
//              UNLOCK();
//              if ( routine_->enabled() ) {
//                emit parameterChanged();
//              }
//            }
//          }
//        });
//
//    return ctl;
//  }
//
//  template<class PropType>
//  QNumberEditBox * add_numeric_box(const QString & label,
//      const PropType & (RoutineType::*getfn)() const,
//      void (RoutineType::*setfn)(const PropType &))
//  {
//    QNumberEditBox * ctl = new QNumberEditBox();
//    ctlform->addRow(label, ctl);
//    QObject::connect(ctl, &QLineEditBox::textChanged,
//        [this, ctl, getfn, setfn]() {
//          if ( routine_ && !updatingControls() ) {
//            PropType value = (routine_.get()->*getfn)();
//            if ( fromString(ctl->text(), &value) && (routine_.get()->*getfn)() != value ) {
//              LOCK();
//              (routine_.get()->*setfn)(value);
//              UNLOCK();
//              if ( routine_->enabled() ) {
//                emit parameterChanged();
//              }
//            }
//          }
//        });
//
//    return ctl;
//  }
//
//  QLineEditBox * add_textbox(const QString & label,
//      const std::string & (RoutineType::*getfn)() const,
//      void (RoutineType::*setfn)(const std::string & ))
//  {
//    QLineEditBox * ctl = new QLineEditBox();
//    ctlform->addRow(label, ctl);
//    QObject::connect(ctl, &QLineEditBox::textChanged,
//        [this, ctl, getfn, setfn]() {
//          if ( routine_ && !updatingControls() ) {
//            LOCK();
//            (routine_.get()->*setfn)(ctl->text().toStdString());
//            UNLOCK();
//            if ( routine_->enabled() ) {
//              emit parameterChanged();
//            }
//          }
//        });
//
//    return ctl;
//  }
//
//
//  template<class ComboboxType, class PropType>
//  ComboboxType * add_enum_combobox(const QString & label,
//      PropType (RoutineType::*getfn)() const,
//      void (RoutineType::*setfn)(PropType))
//  {
//    ComboboxType * ctl = new ComboboxType(this);
//    ctlform->addRow(label, ctl);
//    QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
//        [this, ctl, getfn, setfn]() {
//          if ( routine_ && !updatingControls() ) {
//            PropType value = ctl->currentItem();
//            if ( (routine_.get()->*getfn)() != value ) {
//              LOCK();
//              (routine_.get()->*setfn)(value);
//              UNLOCK();
//              if ( routine_->enabled() ) {
//                emit parameterChanged();
//              }
//            }
//          }
//        });
//
//    return ctl;
//  }
//
//  template<class WidgetType>
//  WidgetType * add_widget(const QString & label, WidgetType * widget) {
//    if ( label.isEmpty() ) {
//      ctlform->addRow(widget);
//    }
//    else {
//      ctlform->addRow(label, widget);
//    }
//    return widget;
//  }
//
//protected:
//  RoutinePtr routine_;
//};

#endif /* __QImageProcessorRoutineSettings_h__ */
