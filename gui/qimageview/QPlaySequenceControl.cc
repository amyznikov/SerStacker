/*
 * QPlaySequenceControl.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QPlaySequenceControl.h"

QPlaySequenceControl::QPlaySequenceControl(QWidget * parent)
  : Base(parent)
{
  QBoxLayout * layout = new QHBoxLayout(this);

  layout->setContentsMargins(4, 0, 0, 4);

  playButton = new QToolButton(this);
  playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  connect(playButton, &QAbstractButton::clicked,
      this, &ThisClass::onPlayClicked);

  curposSlider = new QSlider(Qt::Horizontal, this);
  curposSlider->setRange(0, 0);
  curposSlider->setValue(0);
  curposSlider->setSingleStep(1);
  connect(curposSlider, &QSlider::valueChanged,
      this, &ThisClass::onSliderValueChanged);

  curposLabel = new QLabel("");

  layout->addWidget(playButton);
  layout->addWidget(curposSlider, 100);
  layout->addWidget(curposLabel, 0);
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
      timerId = startTimer(100, Qt::CoarseTimer);
      playButton->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
      break;
    }
    break;

  case Playing :
    switch ( currentState = state ) {
    case Stopped :  // playing -> stopped
      killTimer(timerId);
      timerId = 0;
      playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
      break;
    case Playing :  // playing -> playing
      break;
    }
    break;
  }
}

void QPlaySequenceControl::setSeekRange(int min,  int max)
{
  disableEmitSignals = true;
  curposSlider->setRange(min, max);
  disableEmitSignals = false;
  updateCurposLabel();
}

void QPlaySequenceControl::setCurpos(int pos)
{
  disableEmitSignals = true;
  curposSlider->setValue(pos);
  disableEmitSignals = false;
  updateCurposLabel();
}


void QPlaySequenceControl::onPlayClicked()
{
  if ( currentState == Playing ) {
    setState(Stopped);
  }
  else {
    if ( curposSlider->value() >= curposSlider->maximum() ) {
      curposSlider->setValue(0);
    }
    setState(Playing);
  }
}

void QPlaySequenceControl::onSliderValueChanged(int newpos)
{
  updateCurposLabel();
  if ( !disableEmitSignals ) {
    emit onSeek(newpos);
  }
}

void QPlaySequenceControl::updateCurposLabel()
{
  curposLabel->setText(QString("%1/%2").
      arg(curposSlider->value()).
      arg(curposSlider->maximum() + 1));
}

void QPlaySequenceControl::timerEvent(QTimerEvent *event)
{
  if ( currentState == Playing ) {
    if ( curposSlider->value() < curposSlider->maximum() ) {
      curposSlider->setValue(curposSlider->value() + 1);
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

