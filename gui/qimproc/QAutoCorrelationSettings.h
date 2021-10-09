/*
 * QAutoCorrelationSettings.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QAutoCorrelationSettings_h__
#define __QAutoCorrelationSettings_h__

#include <core/improc/c_auto_correlation_routine.h>
#include "QImageProcessorSelector.h"

class QAutoCorrelationSettings
  : public QImageProcessorRoutineSettings<c_auto_correlation_routine>
{
  Q_OBJECT;
public:

  typedef QAutoCorrelationSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QAutoCorrelationSettings(const c_auto_correlation_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;
};

#endif /* __QAutoCorrelationSettings_h__ */
