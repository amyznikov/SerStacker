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

  void set_polysharp_routine(const c_radial_polysharp_routine::ptr & routine);
  const c_radial_polysharp_routine::ptr & polysharp_routine() const;

protected:
  QSize sizeHint() const override;
  void paintEvent(QPaintEvent *event) override;

protected:
  c_radial_polysharp_routine::ptr routine_;
};

class QRadialPolySharpSettings :
    public QImageProcessorRoutineSettings
{
public:
  typedef QRadialPolySharpSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;


  QRadialPolySharpSettings(const c_radial_polysharp_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void setup_controls() override;
  void onupdatecontrols() override;

protected:
  QRadialPolyProfileView * profileView_ = Q_NULLPTR;
  QLineEditBox * coeffs_ctl = Q_NULLPTR;

};

#endif /* __QRadialPolySharpSettings_h__ */
