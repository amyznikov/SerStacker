/*
 * QRadialPolySharpSettings.h
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QRadialPolySharpSettings_h__
#define __QRadialPolySharpSettings_h__

//#include "QImageProcessorRoutineSettings.h"
#include "QImageProcessorChainEditor.h"
#include <core/improc/generic/c_radial_polysharp_routine.h>

class QRadialPolyProfileView
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QRadialPolyProfileView ThisClass;
  typedef QWidget Base;

  QRadialPolyProfileView(QWidget * parent = nullptr);

  void set_polysharp_routine(const c_radial_polysharp_routine::ptr & routine);
  const c_radial_polysharp_routine::ptr & polysharp_routine() const;

protected:
  QSize sizeHint() const override;
  void paintEvent(QPaintEvent *event) override;

protected:
  c_radial_polysharp_routine::ptr routine_;
};

class QRadialPolySharpSettings :
    public QImageProcessorSettingsControl
{
public:
  typedef QRadialPolySharpSettings ThisClass;
  typedef QImageProcessorSettingsControl Base;

  QRadialPolySharpSettings(const c_radial_polysharp_routine::ptr & processor,
      QWidget * parent = nullptr);

protected:
  void setupControls() override;
  void onupdatecontrols() override;

protected:
  QRadialPolyProfileView * profileView_ = nullptr;
  QLineEditBox * coeffs_ctl = nullptr;

};

#endif /* __QRadialPolySharpSettings_h__ */
