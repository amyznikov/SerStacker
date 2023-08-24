/*
 * QJovianEllipseSettings.h
 *
 *  Created on: Aug 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QJovianEllipseSettings_h__
#define __QJovianEllipseSettings_h__

//#include "QImageProcessorRoutineSettings.h"
#include "QImageProcessorChainEditor.h"
#include <gui/qjovian/QJovianEllipseDetectorSettings.h>
#include <core/improc/feature2d/c_fit_jovian_ellipse_routine.h>

class QJovianEllipseSettings :
    public QImageProcessorSettingsControl
{
public:
  typedef QJovianEllipseSettings ThisClass;
  typedef QImageProcessorSettingsControl Base;
  typedef c_fit_jovian_ellipse_routine::display_type DisplayType;
  typedef QEnumComboBox<DisplayType> DisplayTypeCombo;

  QJovianEllipseSettings(const c_fit_jovian_ellipse_routine::ptr & processor,
      QWidget * parent = nullptr);

protected:
  void setupControls() override;
  void onupdatecontrols() override;

protected:
  QJovianEllipseDetectorSettings * settings_ctl = nullptr;
  DisplayTypeCombo * display_ctl = nullptr;
};

#endif /* __QJovianEllipseSettings_h__ */
