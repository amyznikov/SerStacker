/*
 * QPlaySequenceControl.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QPlaySequenceControl_h__
#define __QPlaySequenceControl_h__

#include <QtWidgets/QtWidgets>

class QPlaySequenceControl :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QPlaySequenceControl ThisClass;
  typedef QWidget Base;

  enum State {
      Stopped,
      Playing,
  };

  QPlaySequenceControl(QWidget * parent = nullptr);

  void setState(State state);
  State state() const;

  void setSeekRange(int min,  int max);
  void setCurpos(int pos);
  int curPos() const;

Q_SIGNALS:
  void onSeek(int pos);

protected:
  void timerEvent(QTimerEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected slots:
  void onPlayClicked();
  void onSpinValueChanged(int newpos);
  void onSliderValueChanged(int newpos);
  void updateCurposLabel();

protected:
  QAbstractButton * playButton_ctl = nullptr;
  QSpinBox * curposSpin_ctl = nullptr;
  QSlider * curposSlider_ctl = nullptr;
  QLabel * curposLabel_ctl = nullptr;
  QComboBox * rateBox_ctl = nullptr;
  State currentState = Stopped;
  int timerId = 0;
};

#endif /* __QPlaySequenceControl_h__ */
