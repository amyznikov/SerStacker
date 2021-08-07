/*
 * QHistogramWhiteBalanceSettings.h
 *
 *  Created on: Aug 7, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QHistogramWhiteBalanceSettings_h__
#define __QHistogramWhiteBalanceSettings_h__

#include "QImageProcessorChainEditor.h"
#include <core/improc/c_histogram_white_balance_routine.h>

class QHistogramWhiteBalanceSettings
    : public QImageProcessorRoutineSettings<c_histogram_white_balance_routine>
{
  Q_OBJECT;
public:
  typedef QHistogramWhiteBalanceSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;


  QHistogramWhiteBalanceSettings(const c_histogram_white_balance_routine::ptr & routine,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * lclip_ctl = Q_NULLPTR;
  QNumberEditBox * hclip_ctl = Q_NULLPTR;
  QCheckBox * enable_threshold_ctl = Q_NULLPTR;
  QNumberEditBox * threshold_ctl = Q_NULLPTR;
};

#endif /* __QHistogramWhiteBalanceSettings_h__ */
