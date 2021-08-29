/*
 * QInPaintSettings.h
 *
 *  Created on: Aug 29, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInPaintSettings_h__
#define __QInPaintSettings_h__

#include <core/improc/c_inpaint_routine.h>
#include "QImageProcessorSelector.h"


class QInPaintSettings
  : public QImageProcessorRoutineSettings<c_inpaint_routine>
{
  Q_OBJECT;
public:
  typedef QInPaintSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QInPaintSettings(const c_inpaint_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QLabel * label_ctl = Q_NULLPTR;
};


#endif /* __QInPaintSettings_h__ */
