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


#endif /* __QImageProcessorRoutineSettings_h__ */
