/*
 * QRangeNormalizeSettings.h
 *
 *  Created on: Aug 7, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRangeNormalizeSettings_h__
#define __QRangeNormalizeSettings_h__

#include "QImageProcessorChainEditor.h"
#include <core/improc/c_range_normalize_routine.h>


class QRangeNormalizeSettings
    : public QImageProcessorRoutineSettings<c_range_normalize_routine>
{
  Q_OBJECT;
public:
  typedef QRangeNormalizeSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;


  QRangeNormalizeSettings(const c_range_normalize_routine::ptr & routine,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QCheckBox * auto_input_range_ctl = Q_NULLPTR;
  QNumberEditBox * input_min_ctl = Q_NULLPTR;
  QNumberEditBox * input_max_ctl = Q_NULLPTR;
  QNumberEditBox * output_min_ctl = Q_NULLPTR;
  QNumberEditBox * output_max_ctl = Q_NULLPTR;
};

#endif /* __QRangeNormalizeSettings_h__ */
