/*
 * QAutoClipSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QAutoClipSettings_h__
#define __QAutoClipSettings_h__


#include "QImageProcessorCollectionSettings.h"
#include <core/improc/c_autoclip_routine.h>

class QAutoClipSettings
  : public QImageProcessorRoutineSettings<c_autoclip_routine>
{
  Q_OBJECT;
public:
  typedef QAutoClipSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QAutoClipSettings(const c_autoclip_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * lclip_ctl = Q_NULLPTR;
  QNumberEditBox * hclip_ctl = Q_NULLPTR;
};


#endif /* __QAutoClipSettings_h__ */
