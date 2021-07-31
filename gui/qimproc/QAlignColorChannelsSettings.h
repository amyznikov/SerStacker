/*
 * QAlignColorChannelsSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QAlignColorChannelsSettings_h__
#define __QAlignColorChannelsSettings_h__

#include "QImageProcessorCollectionSettings.h"
#include <core/improc/c_align_color_channels_routine.h>
#include <core/debug.h>

class QAlignColorChannelsSettings
    : public QImageProcessorRoutineSettings<c_align_color_channels_routine>
{
  Q_OBJECT;
public:
  typedef QAlignColorChannelsSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;


  QAlignColorChannelsSettings(const c_align_color_channels_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * reference_channel_ctl = Q_NULLPTR;
};


#endif /* __QAlignColorChannelsSettings_h__ */
