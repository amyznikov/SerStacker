/*
 * QGlViewPlanarGridSettings.h
 *
 *  Created on: Feb 8, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGlViewPlanarGridSettings_h__
#define __QGlViewPlanarGridSettings_h__

#include "QGLView.h"
#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>

class QGLViewPlanarGridSettings :
    public QSettingsWidgetTemplate<QGLView::PlanarGridOptions>
{
public:
  typedef QGLViewPlanarGridSettings ThisClass;
  typedef QSettingsWidgetTemplate<QGLView::PlanarGridOptions> Base;

  QGLViewPlanarGridSettings(QWidget * parent = nullptr);

protected:
  QCheckBox * visible_ctl = nullptr;
  QLineEditBox * name_ctl = nullptr;
  QNumericBox * step_ctl = nullptr;
  QNumericBox * max_distance_ctl = nullptr;
  QNumericBox * rotation_ctl = nullptr;
  QNumericBox * translation_ctl = nullptr;
  QColorPickerButton * gridColor_ctl = nullptr;
  QIntegerSliderSpinBox * gridOpaqueness_ctl = nullptr;
  QColorPickerButton * fillColor_ctl = nullptr;
  QIntegerSliderSpinBox * fillOpaqueness_ctl = nullptr;
};

class QGLViewPlanarGridSettingsDialogBox :
    public QSettingsDialogBoxTemplate<QGLViewPlanarGridSettings>
{
public:
  typedef QGLViewPlanarGridSettingsDialogBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QGLViewPlanarGridSettings> Base;

  QGLViewPlanarGridSettingsDialogBox(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags()) :
      ThisClass("", parent, flags)
  {
  }

  QGLViewPlanarGridSettingsDialogBox(const QString & title, QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags()) :
    Base(title, parent, flags)
  {
  }
};

#endif /* __QGlViewPlanarGridSettings_h__ */
