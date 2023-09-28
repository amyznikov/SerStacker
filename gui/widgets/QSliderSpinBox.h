/*
 * QSliderSpinBox.h
 *
 *  Created on: Feb 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSliderSpinBox_h__
#define __QSliderSpinBox_h__

#include <QtWidgets/QtWidgets>
#include <type_traits>

class QIntegerSliderSpinBox;
class QDoubleSliderSpinBox;

template<bool IsFloatingPointType>
struct QSliderSpinBoxTypeSelect {};
template<>
struct QSliderSpinBoxTypeSelect<false> {
  typedef QSpinBox SpinBoxType;
  typedef QIntegerSliderSpinBox SliderSpinBoxType;
};
template<>
struct QSliderSpinBoxTypeSelect<true> {
  typedef QDoubleSpinBox SpinBoxType;
  typedef QDoubleSliderSpinBox SliderSpinBoxType;
};


template<class T>
class QSliderSpinBox:
    public QWidget
{
public:
  typedef QSliderSpinBox ThisClass;
  typedef QWidget Base;
  typedef typename QSliderSpinBoxTypeSelect<std::is_floating_point_v<T>>::SliderSpinBoxType Type;
  typedef typename QSliderSpinBoxTypeSelect<std::is_floating_point_v<T>>::SpinBoxType SpinBoxType;


  class c_updating_controls_lock
  {
    QSliderSpinBox * obj;
  public:
    c_updating_controls_lock(QSliderSpinBox * _obj) : obj(_obj)
    {
      ++obj->updatingControls_;
    }
    ~c_updating_controls_lock()
    {
      if( --obj->updatingControls_ < 0 ) {
        obj->updatingControls_ = 0;
      }
    }
  };



  QSliderSpinBox(QWidget * parent = nullptr) :
      Base(parent)
  {
    lv_ = new QVBoxLayout(this);
    lv_->addWidget(spinBox_ctl = new SpinBoxType(this));
    lv_->addWidget(slider_ctl = new QSlider(Qt::Orientation::Horizontal, this));

    spinBox_ctl->setFocusPolicy(Qt::StrongFocus);
    spinBox_ctl->setKeyboardTracking(false);
    slider_ctl->setFocusPolicy(Qt::StrongFocus);
  }

  QAbstractSpinBox::StepType stepType() const
  {
    return spinBox_ctl->stepType();
  }

  void setStepType(QAbstractSpinBox::StepType stepType)
  {
    spinBox_ctl->setStepType(stepType);
  }

  QSlider::TickPosition tickPosition() const
  {
    return slider_ctl->tickPosition();
  }

  void setTickPosition(QSlider::TickPosition position)
  {
    slider_ctl->setTickPosition(position);
  }

  T minimum() const
  {
    return spinBox_ctl->minimum();
  }

  void setMinimum(T min)
  {
    c_updating_controls_lock lock(this);
    spinBox_ctl->setMinimum(min);
    slider_ctl->setMinimum(min);
  }

  T maximum() const
  {
    return (T)spinBox_ctl->maximum();
  }

  void setMaximum(T max)
  {
    c_updating_controls_lock lock(this);
    spinBox_ctl->setMaximum(max);
    slider_ctl->setMaximum(max);
  }

  T tickInterval() const
  {
    return slider_ctl->tickInterval();
  }

  void setTickInterval(T val)
  {
    slider_ctl->setTickInterval(val);
  }

protected:
  QVBoxLayout * lv_ = nullptr;
  SpinBoxType * spinBox_ctl = nullptr;
  QSlider * slider_ctl = nullptr;
  int updatingControls_ = 0;
};


class QIntegerSliderSpinBox:
    public QSliderSpinBox<int>
{
  Q_OBJECT;
public:
  typedef QIntegerSliderSpinBox ThisClass;
  typedef QSliderSpinBox<int> Base;

  QIntegerSliderSpinBox(QWidget * parent = nullptr) : Base(parent)
  {
    connect(spinBox_ctl, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
        [this](int value) {
          if ( !updatingControls_ ) {
            c_updating_controls_lock lock(this);
            slider_ctl->setValue(value);
            Q_EMIT valueChanged(value);
          }
        });

    connect(slider_ctl, &QSlider::valueChanged,
        [this](int value) {
          if ( !updatingControls_ ) {
            c_updating_controls_lock lock(this);
            spinBox_ctl->setValue(value);
            Q_EMIT valueChanged(value);
          }
        });
  }

  void setRange(int min, int max)
  {
    c_updating_controls_lock lock(this);
    spinBox_ctl->setRange(min, max);
    slider_ctl->setRange(min, max);
  }

  int value() const
  {
    return spinBox_ctl->value();
  }

  void setValue(int val)
  {
    c_updating_controls_lock lock(this);
    spinBox_ctl->setValue(val);
    slider_ctl->setValue(val);
  }


Q_SIGNALS:
  void valueChanged(int);
};


class QDoubleSliderSpinBox:
    public QSliderSpinBox<double>
{
  Q_OBJECT;
public:
  typedef QDoubleSliderSpinBox ThisClass;
  typedef QSliderSpinBox<double> Base;

  static inline constexpr double slider_eps()
  {
    return 1e-3;
  }

  QDoubleSliderSpinBox(QWidget * parent = nullptr) : Base(parent)
  {
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    spinBox_ctl->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
#endif

    connect(spinBox_ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
        [this](double value) {
          if ( !updatingControls_ ) {
            c_updating_controls_lock lock(this);
            slider_ctl->setValue((value - spinBox_ctl->minimum())/slider_eps());
            Q_EMIT valueChanged(value);
          }
        });

    connect(slider_ctl, &QSlider::valueChanged,
        [this](int value) {
          if ( !updatingControls_ ) {
            c_updating_controls_lock lock(this);
            spinBox_ctl->setValue(spinBox_ctl->minimum() + value * slider_eps());
            Q_EMIT valueChanged(spinBox_ctl->value());
          }
        });
  }

  void setRange(int min, int max)
  {
    c_updating_controls_lock lock(this);
    spinBox_ctl->setRange(min, max);
    slider_ctl->setRange(0, (max - min)/slider_eps());
    slider_ctl->setSingleStep(1);
  }

  void setSingleStep(double val)
  {
    spinBox_ctl->setSingleStep(val);
  }

  double singleStep() const
  {
    return spinBox_ctl->singleStep();
  }

  double value() const
  {
    return spinBox_ctl->value();
  }

  void setValue(double value)
  {
    c_updating_controls_lock lock(this);
    spinBox_ctl->setValue(value);
    slider_ctl->setValue((spinBox_ctl->value() - spinBox_ctl->minimum())/slider_eps());
  }


Q_SIGNALS:
  void valueChanged(double value);
};


//  // the partial specialization of A is enabled via a template parameter
//  template<class A, class Enable = void>
//  struct QSpinBoxTypeSelect {}; // primary template
//
//  template<class A>
//  struct QSpinBoxTypeSelect<A, typename std::enable_if<std::is_floating_point<A>::value>::type> {
//    typedef QDoubleSpinBox SpinBoxType;
//  };
//  template<class A>
//  struct QSpinBoxTypeSelect<A, typename std::enable_if<std::is_integral<A>::value>::type> {
//    typedef QSpinBox SpinBoxType;
//  };



#endif /* __QSliderSpinBox_h__ */
