/*
 * QMtfSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMtfSettings_h__
#define __QMtfSettings_h__

#include <gui/qmtfcontrols/QMtfControl.h>
#include <core/improc/c_mtf_routine.h>
#include "QImageProcessorSelector.h"

class QMtfSettings
  : public QImageProcessorRoutineSettings<c_mtf_routine>
{
  Q_OBJECT;
public:
  typedef QMtfSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  QMtfSettings(const c_mtf_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QMtfControl * mtf_ctl = Q_NULLPTR;
};

#endif /* __QMtfSettings_h__ */