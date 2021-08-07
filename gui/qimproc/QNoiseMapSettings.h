/*
 * QNoiseMapSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QNoiseMapSettings_h__
#define __QNoiseMapSettings_h__


#include <core/improc/c_noisemap_routine.h>
#include "QImageProcessorSelector.h"


class QNoiseMapSettings
  : public QImageProcessorRoutineSettings<c_noisemap_routine>
{
  Q_OBJECT;
public:
  typedef QNoiseMapSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QNoiseMapSettings(const c_noisemap_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
};


#endif /* __QNoiseMapSettings_h__ */
