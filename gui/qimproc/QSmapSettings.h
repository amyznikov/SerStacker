/*
 * QSmapSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSmapSettings_h__
#define __QSmapSettings_h__

#include "QImageProcessorCollectionSettings.h"
#include <core/improc/c_smap_routine.h>

class QSmapSettings
  : public QImageProcessorRoutineSettings<c_smap_routine>
{
  Q_OBJECT;
public:
  typedef QSmapSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QSmapSettings(const c_smap_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * minv_ctl = Q_NULLPTR;
  QNumberEditBox * scale_ctl = Q_NULLPTR;
};



#endif /* __QSmapSettings_h__ */
