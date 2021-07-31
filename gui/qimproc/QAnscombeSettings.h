/*
 * QAnscombeSettings.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QAnscombeSettings_h__
#define __QAnscombeSettings_h__

#include "QImageProcessorCollectionSettings.h"
#include <core/improc/c_anscombe_routine.h>

class QAnscombeSettings
  : public QImageProcessorRoutineSettings<c_anscombe_routine>
{
  Q_OBJECT;
public:
  typedef QAnscombeSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;

  static QString toString(enum anscombe_method v) {
    return QString::fromStdString(toStdString(v));
  }

  static enum anscombe_method fromString(const QString  & s, enum anscombe_method defval ) {
    return fromStdString(s.toStdString(), defval);
  }

  class QAnscombeMethodCombo :
      public QEnumComboBox<anscombe_method>
  {
  public:
    typedef QEnumComboBox<anscombe_method> Base;
    QAnscombeMethodCombo(QWidget * parent = Q_NULLPTR)
        : Base(parent, anscombe_methods)
      {}
  };

  QAnscombeSettings(const c_anscombe_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QAnscombeMethodCombo * method_ctl = Q_NULLPTR;
};


#endif /* __QAnscombeSettings_h__ */
