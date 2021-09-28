/*
 * QRadialPolySharpSettings.h
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRadialPolySharpSettings_h__
#define __QRadialPolySharpSettings_h__

#include "QImageProcessorRoutineSettings.h"
#include <core/improc/c_radial_polysharp_routine.h>

class QRadialPolyProfileView
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QRadialPolyProfileView ThisClass;
  typedef QWidget Base;

  QRadialPolyProfileView(QWidget * parent = Q_NULLPTR);

  void set_radial_polysharp_routine(const c_radial_polysharp_routine::ptr & p);
  const c_radial_polysharp_routine::ptr & radial_polysharp_routine() const;

protected:
  QSize sizeHint() const override;
  void paintEvent(QPaintEvent *event) override;

protected:
  c_radial_polysharp_routine::ptr routine_;
};

class QRadialPolySharpSettings :
    public QImageProcessorRoutineSettings<c_radial_polysharp_routine>
{
public:
  typedef QRadialPolySharpSettings ThisClass;
  typedef QImageProcessorRoutineSettings<c_radial_polysharp_routine> Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;


  QRadialPolySharpSettings(const c_radial_polysharp_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QRadialPolyProfileView * profileView_ = Q_NULLPTR;
  QLineEditBox * coeffs_ctl = Q_NULLPTR;

};

#endif /* __QRadialPolySharpSettings_h__ */
