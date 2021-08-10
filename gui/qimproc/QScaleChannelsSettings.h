/*
 * QScaleChannelsSettings.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QScaleChannelsSettings_h__
#define __QScaleChannelsSettings_h__

#include "QImageProcessorSelector.h"
#include <core/improc/c_scale_channels_routine.h>

class QScaleChannelsSettings
  : public QImageProcessorRoutineSettings<c_scale_channels_routine>
{
  Q_OBJECT;
public:
  typedef QScaleChannelsSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QScaleChannelsSettings(const c_scale_channels_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * bias_r_ctl = Q_NULLPTR;
  QNumberEditBox * stretch_r_ctl = Q_NULLPTR;

  QNumberEditBox * bias_g_ctl = Q_NULLPTR;
  QNumberEditBox * stretch_g_ctl = Q_NULLPTR;

  QNumberEditBox * bias_b_ctl = Q_NULLPTR;
  QNumberEditBox * stretch_b_ctl = Q_NULLPTR;

  QNumberEditBox * bias_a_ctl = Q_NULLPTR;
  QNumberEditBox * stretch_a_ctl = Q_NULLPTR;
};

#endif /* __QScaleChannelsSettings_h__ */
