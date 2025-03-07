/*
 * QPlaySequenceControl.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QPlaySequenceControl.h"
#include <gui/widgets/QSignalsBock.h>
#include <core/debug.h>

QPlaySequenceControl::QPlaySequenceControl(QWidget * parent) :
  Base(parent)
{
  QBoxLayout * layout = new QHBoxLayout(this);

  layout->setContentsMargins(4, 0, 0, 4);

  playButton_ctl = new QToolButton(this);
  playButton_ctl->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  connect(playButton_ctl, &QAbstractButton::clicked,
      this, &ThisClass::onPlayClicked);

  curposSpin_ctl = new QSpinBox(this);
  curposSpin_ctl->setKeyboardTracking(false);
  curposSpin_ctl->setAccelerated(true);
  curposSpin_ctl->setRange(0, 0);
  curposSpin_ctl->setButtonSymbols(QSpinBox::ButtonSymbols::UpDownArrows);
  connect(curposSpin_ctl, SIGNAL(valueChanged(int)),
      this, SLOT(onSpinValueChanged(int)));


  curposSlider_ctl = new QSlider(Qt::Horizontal, this);
  curposSlider_ctl->setRange(0, 0);
  curposSlider_ctl->setValue(0);
  curposSlider_ctl->setSingleStep(1);
  connect(curposSlider_ctl, &QSlider::valueChanged,
      this, &ThisClass::onSliderValueChanged);

  curposLabel_ctl = new QLabel("");

  layout->addWidget(curposSpin_ctl);
  layout->addWidget(playButton_ctl);
  layout->addWidget(curposSlider_ctl, 100);
  layout->addWidget(curposLabel_ctl, 0);
}

QPlaySequenceControl::State QPlaySequenceControl::state() const
{
  return currentState;
}


void QPlaySequenceControl::setState(State state)
{
  switch ( currentState ) {

  case Stopped :
    switch ( currentState = state ) {
    case Stopped :  // stopped -> stopped
      break;
    case Playing :  // stopped -> playing
      timerId = startTimer(75, Qt::CoarseTimer);
      playButton_ctl->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
      break;
    }
    break;

  case Playing :
    switch ( currentState = state ) {
    case Stopped :  // playing -> stopped
      killTimer(timerId);
      timerId = 0;
      playButton_ctl->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
      break;
    case Playing :  // playing -> playing
      break;
    }
    break;
  }
}

void QPlaySequenceControl::setSeekRange(int min,  int max)
{
  QSignalsBock block1(curposSlider_ctl);
  QSignalsBock block2(curposSpin_ctl);
  curposSlider_ctl->setRange(min, max);
  curposSpin_ctl->setRange(min, max);
  updateCurposLabel();
}

void QPlaySequenceControl::setCurpos(int pos)
{
  QSignalsBock block1(curposSlider_ctl);
  QSignalsBock block2(curposSpin_ctl);
  curposSpin_ctl->setValue(pos);
  curposSlider_ctl->setValue(curposSpin_ctl->value());
}

int QPlaySequenceControl::curPos() const
{
  return curposSpin_ctl->value();
}


void QPlaySequenceControl::onPlayClicked()
{
  if ( currentState == Playing ) {
    setState(Stopped);
  }
  else {
    if ( curposSlider_ctl->value() >= curposSlider_ctl->maximum() ) {
      curposSlider_ctl->setValue(0);
    }
    setState(Playing);
  }
}

void QPlaySequenceControl::onSpinValueChanged(int newpos)
{
  QSignalBlocker block(curposSlider_ctl);
  curposSlider_ctl->setValue(newpos);
  Q_EMIT onSeek(newpos);

  //QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}

void QPlaySequenceControl::onSliderValueChanged(int newpos)
{
  curposSlider_ctl->setMouseTracking(false);

  QSignalBlocker block(curposSpin_ctl);
  curposSpin_ctl->setValue(newpos);
  Q_EMIT onSeek(newpos);

  //QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
  curposSlider_ctl->setMouseTracking(true);
}



void QPlaySequenceControl::updateCurposLabel()
{
  curposLabel_ctl->setText(QString("%1").
      arg(curposSlider_ctl->maximum() + 1));
}

void QPlaySequenceControl::timerEvent(QTimerEvent *event)
{
  if ( currentState == Playing ) {
    if ( curposSlider_ctl->value() < curposSlider_ctl->maximum() ) {
      curposSlider_ctl->setValue(curposSlider_ctl->value() + 1);
    }
    else {
      setState(Stopped);
    }
  }
}

void QPlaySequenceControl::hideEvent(QHideEvent *event)
{
  setState(Stopped);
}

