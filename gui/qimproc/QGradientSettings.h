/*
 * QGradientSettings.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#ifndef __QGradientSettings_h__
#define __QGradientSettings_h__

#include <core/improc/c_gradient_routine.h>
#include "QImageProcessorSelector.h"

class QGradientSettings
  : public QImageProcessorRoutineSettings<c_gradient_routine>
{
  Q_OBJECT;
public:
  typedef QGradientSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QGradientSettings(const c_gradient_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;
};

#endif /* __QGradientSettings_h__ */
