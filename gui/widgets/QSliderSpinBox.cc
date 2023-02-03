/*
 * QSliderSpinBox.cc
 *
 *  Created on: Feb 2, 2023
 *      Author: amyznikov
 */

#include "QSliderSpinBox.h"


//class QSliderSpinBox:
//    public QWidget
//{
//  Q_OBJECT;
//public:
//  typedef QSliderSpinBox ThisClass;
//  typedef QWidget Base;
//
//  QSliderSpinBox(QWidget * parent = nullptr);
//
//  int value() const;
//  int minimum() const;
//  int maximum() const;
//
//  void setRange(int min, int max);
//
//  QSpinBox::StepType stepType() const;
//  void setStepType(QSpinBox::StepType stepType);
//
//  QSlider::TickPosition tickPosition() const;
//  void setTickPosition(QSlider::TickPosition position);
//
//  int tickInterval() const;
//  void setTickInterval(int ti);
//
//public Q_SLOTS:
//  void setValue(int val);
//  void setMinimum(int min);
//  void setMaximum(int max);
//
//Q_SIGNALS:
//  void valueChanged(int);
//
//protected:
//  QVBoxLayout * lv_ = nullptr;
//  QSpinBox * spinBox_ctl = nullptr;
//  QSlider * slider_ctl = nullptr;
//  int updatingControls_ = 0;
//
//  class c_updating_controls_lock
//  {
//    QSliderSpinBox * obj;
//  public:
//    c_updating_controls_lock(QSliderSpinBox * _obj) : obj(_obj)
//    {
//      ++obj->updatingControls_;
//    }
//    ~c_updating_controls_lock()
//    {
//      if( --obj->updatingControls_ < 0 ) {
//        obj->updatingControls_ = 0;
//      }
//    }
//  };
//
//};
//
//QSliderSpinBox::QSliderSpinBox(QWidget * parent) :
//    Base(parent)
//{
//  lv_ = new QVBoxLayout(this);
//  lv_->addWidget(spinBox_ctl = new QSpinBox(this));
//  lv_->addWidget(slider_ctl = new QSlider(Qt::Orientation::Horizontal, this));
//
//  spinBox_ctl->setFocusPolicy(Qt::StrongFocus);
//  //spinBox_ctl->setMouseTracking(false);
//  spinBox_ctl->setKeyboardTracking(false);
//
//  slider_ctl->setFocusPolicy(Qt::StrongFocus);
//  //slider_ctl->setMouseTracking(false);
//
//  connect(spinBox_ctl, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
//      [this](int value) {
//        if ( !updatingControls_ ) {
//          c_updating_controls_lock lock(this);
//          slider_ctl->setValue(value);
//          Q_EMIT valueChanged(value);
//        }
//      });
//
//  connect(slider_ctl, &QSlider::valueChanged,
//      [this](int value) {
//        if ( !updatingControls_ ) {
//          c_updating_controls_lock lock(this);
//          spinBox_ctl->setValue(value);
//          Q_EMIT valueChanged(value);
//        }
//      });
//}
//
//
//int QSliderSpinBox::value() const
//{
//  return spinBox_ctl->value();
//}
//
//void QSliderSpinBox::setValue(int val)
//{
//  c_updating_controls_lock lock(this);
//  spinBox_ctl->setValue(val);
//  slider_ctl->setValue(val);
//}
//
//int QSliderSpinBox::minimum() const
//{
//  return spinBox_ctl->minimum();
//}
//
//void QSliderSpinBox::setMinimum(int min)
//{
//  c_updating_controls_lock lock(this);
//  spinBox_ctl->setMinimum(min);
//  slider_ctl->setMinimum(min);
//}
//
//int QSliderSpinBox::maximum() const
//{
//  return spinBox_ctl->maximum();
//}
//
//void QSliderSpinBox::setMaximum(int max)
//{
//  c_updating_controls_lock lock(this);
//  spinBox_ctl->setMaximum(max);
//  slider_ctl->setMaximum(max);
//}
//
//void QSliderSpinBox::setRange(int min, int max)
//{
//  c_updating_controls_lock lock(this);
//  spinBox_ctl->setRange(min, max);
//  slider_ctl->setRange(min, max);
//}
//
//QSpinBox::StepType QSliderSpinBox::stepType() const
//{
//  return spinBox_ctl->stepType();
//}
//
//void QSliderSpinBox::setStepType(QSpinBox::StepType stepType)
//{
//  spinBox_ctl->setStepType(stepType);
//}
//
//QSlider::TickPosition QSliderSpinBox::tickPosition() const
//{
//  return slider_ctl->tickPosition();
//}
//
//void QSliderSpinBox::setTickPosition(QSlider::TickPosition position)
//{
//  slider_ctl->setTickPosition(position);
//}
//
//int QSliderSpinBox::tickInterval() const
//{
//  return slider_ctl->tickInterval();
//}
//
//void QSliderSpinBox::setTickInterval(int ti)
//{
//  slider_ctl->setTickInterval(ti);
//}


