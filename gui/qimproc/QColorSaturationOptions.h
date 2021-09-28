/*
 * QColorSaturationOptions.h
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QColorSaturationOptions_h__
#define __QColorSaturationOptions_h__

#include "QImageProcessorRoutineSettings.h"
#include <core/improc/c_color_saturation_routine.h>

class QColorSaturationOptions :
    public QImageProcessorRoutineSettings<c_color_saturation_routine>
{
public:
  typedef QColorSaturationOptions ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;


  QColorSaturationOptions(const RoutinePtr & routine,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QLineEditBox * scales_ctl = Q_NULLPTR;
};

#endif /* __QColorSaturationOptions_h__ */
