/*
 * QGaussianFilterSettings.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#ifndef __QGaussianFilterSettings_h__
#define __QGaussianFilterSettings_h__

#include <core/improc/c_gaussian_filter_routine.h>
#include "QImageProcessorRoutineSettings.h"

class QGaussianFilterSettings
  : public QImageProcessorRoutineSettings<c_gaussian_filter_routine>
{
  Q_OBJECT;
public:
  typedef QGaussianFilterSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QGaussianFilterSettings(const c_gaussian_filter_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * sigma_ctl = Q_NULLPTR;

};

#endif /* __QGaussianFilterSettings_h__ */
