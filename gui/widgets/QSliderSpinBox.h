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
#include <gui/widgets/QSignalBlock.h>
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

  QSliderSpinBox(QWidget * parent = nullptr) :
      Base(parent)
  {
    _lv = new QVBoxLayout(this);
    _lv->addWidget(spinBox_ctl = new SpinBoxType(this));
    _lv->addWidget(slider_ctl = new QSlider(Qt::Orientation::Horizontal, this));

    spinBox_ctl->setFocusPolicy(Qt::StrongFocus);
    spinBox_ctl->setKeyboardTracking(false);
    slider_ctl->setFocusPolicy(Qt::StrongFocus);
  }

  void setTooltip(const QString & tooltip )
  {
    QSignalBlock block(spinBox_ctl);
    spinBox_ctl->setTooltip(tooltip);
  }

  QString tooltip() const
  {
    return spinBox_ctl->tooltip();
  }

  QAbstractSpinBox::StepType stepType() const
  {
    QSignalBlock block(spinBox_ctl);
    return spinBox_ctl->stepType();
  }

  void setStepType(QAbstractSpinBox::StepType stepType)
  {
    QSignalBlock block(spinBox_ctl);
    spinBox_ctl->setStepType(stepType);
  }

  QSlider::TickPosition tickPosition() const
  {
    return slider_ctl->tickPosition();
  }

  void setTickPosition(QSlider::TickPosition position)
  {
    QSignalBlock block(spinBox_ctl);
    slider_ctl->setTickPosition(position);
  }

  T minimum() const
  {
    return spinBox_ctl->minimum();
  }

  void setMinimum(T min)
  {
    QSignalBlock block(spinBox_ctl, slider_ctl);
    spinBox_ctl->setMinimum(min);
    slider_ctl->setMinimum(min);
  }

  T maximum() const
  {
    return (T)spinBox_ctl->maximum();
  }

  void setMaximum(T max)
  {
    QSignalBlock block(spinBox_ctl, slider_ctl);
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
  QVBoxLayout * _lv = nullptr;
  SpinBoxType * spinBox_ctl = nullptr;
  QSlider * slider_ctl = nullptr;
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
          QSignalBlock block(slider_ctl);
          slider_ctl->setValue(value);
          Q_EMIT valueChanged(value);
        });

    connect(slider_ctl, &QSlider::valueChanged,
        [this](int value) {
            QSignalBlock block(spinBox_ctl);
            spinBox_ctl->setValue(value);
            Q_EMIT valueChanged(value);
        });
  }

  void setRange(int min, int max)
  {
    QSignalBlock block(spinBox_ctl, slider_ctl);
    spinBox_ctl->setRange(min, max);
    slider_ctl->setRange(min, max);
  }

  int value() const
  {
    return spinBox_ctl->value();
  }

  void setValue(int val)
  {
    QSignalBlock block(spinBox_ctl, slider_ctl);
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

    spinBox_ctl->setDecimals(3);

    connect(spinBox_ctl, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
        [this](double value) {
            QSignalBlock block(slider_ctl);
            slider_ctl->setValue((value - spinBox_ctl->minimum())/slider_eps());
            Q_EMIT valueChanged(value);
        });

    connect(slider_ctl, &QSlider::valueChanged,
        [this](int value) {
            QSignalBlock block(spinBox_ctl);
            spinBox_ctl->setValue(spinBox_ctl->minimum() + value * slider_eps());
            Q_EMIT valueChanged(spinBox_ctl->value());
        });
  }

  void setRange(double min, double max)
  {
    QSignalBlock block(spinBox_ctl, slider_ctl);
    spinBox_ctl->setRange(min, max);
    slider_ctl->setRange(0, (max - min)/slider_eps());
    slider_ctl->setSingleStep(1);
  }

  void setSingleStep(double val)
  {
    QSignalBlock block(spinBox_ctl);
    spinBox_ctl->setSingleStep(val);
  }

  double singleStep() const
  {
    return spinBox_ctl->singleStep();
  }

  void setDecimals(int prec)
  {
    QSignalBlock block(spinBox_ctl);
    spinBox_ctl->setDecimals(prec);
  }

  int decimals() const
  {
    return spinBox_ctl->decimals();
  }

  double value() const
  {
    return spinBox_ctl->value();
  }

  void setValue(double value)
  {
    QSignalBlock block(spinBox_ctl, slider_ctl);
    spinBox_ctl->setValue(value);
    slider_ctl->setValue((spinBox_ctl->value() - spinBox_ctl->minimum())/slider_eps());
  }


Q_SIGNALS:
  void valueChanged(double value);
};




#endif /* __QSliderSpinBox_h__ */
