/*
 * QRangeClipSettings.h
 *
 *  Created on: Aug 6, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRangeClipSettings_h__
#define __QRangeClipSettings_h__

#include "QImageProcessorChainEditor.h"
#include <core/improc/c_rangeclip_routine.h>

class QRangeClipSettings
    : public QImageProcessorRoutineSettings<c_rangeclip_routine>
{
Q_OBJECT ;
public:
  typedef QRangeClipSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {
    }
  } classFactory;

  QRangeClipSettings(const c_rangeclip_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * min_ctl = Q_NULLPTR;
  QNumberEditBox * max_ctl = Q_NULLPTR;
};

#endif /* __QRangeClipSettings_h__ */
