/*
 * QUnsharpMaskSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QUnsharpMaskSettings_h__
#define __QUnsharpMaskSettings_h__

#include <core/improc/c_unsharp_mask_routine.h>
#include "QImageProcessorSelector.h"

class QUnsharpMaskSettings
  : public QImageProcessorRoutineSettings<c_unsharp_mask_routine>
{
  Q_OBJECT;
public:
  typedef QUnsharpMaskSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;


  QUnsharpMaskSettings(const c_unsharp_mask_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * sigma_ctl = Q_NULLPTR;
  QNumberEditBox * alpha_ctl = Q_NULLPTR;
  QNumberEditBox * outmin_ctl = Q_NULLPTR;
  QNumberEditBox * outmax_ctl = Q_NULLPTR;
};



#endif /* __QUnsharpMaskSettings_h__ */
