/*
 * QImageProcessorChainEditor.h
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageProcessorChainEditor_h__
#define __QImageProcessorChainEditor_h__

#include "QImageProcessorRoutineSettings.h"

class QImageProcessorChainEditor :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageProcessorChainEditor ThisClass;
  typedef QSettingsWidget Base;

  QImageProcessorChainEditor(QWidget * parent = Q_NULLPTR);

  void set_current_processor(const c_image_processor::ptr & p);
  const c_image_processor::ptr & current_processor() const;

protected slots:
  void addRoutine(QImageProcessorRoutineSettings * w = Q_NULLPTR);
  void removeRoutine(QImageProcessorRoutineSettings * w);
  void moveUpRoutine(QImageProcessorRoutineSettings * w);
  void moveDownRoutine(QImageProcessorRoutineSettings * w);

protected:
  void onupdatecontrols() override;

protected:
  c_image_processor::ptr current_processor_;
};

#endif /* __QImageProcessorChainEditor_h__ */
