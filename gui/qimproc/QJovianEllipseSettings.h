/*
 * QJovianEllipseSettings.h
 *
 *  Created on: Aug 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QJovianEllipseSettings_h__
#define __QJovianEllipseSettings_h__

#include "QImageProcessorRoutineSettings.h"
#include <gui/qjovian/QJovianEllipseDetectorSettings.h>
#include <core/improc/c_fit_jovian_ellipse_routine.h>

class QJovianEllipseSettings :
    public QImageProcessorRoutineSettings
{
public:
  typedef QJovianEllipseSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;
  typedef c_fit_jovian_ellipse_routine::display_type DisplayType;
  typedef QEnumComboBox<DisplayType> DisplayTypeCombo;

  QJovianEllipseSettings(const c_fit_jovian_ellipse_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void setup_controls() override;
  void onupdatecontrols() override;

protected:
  QJovianEllipseDetectorSettings * settings_ctl = Q_NULLPTR;
  DisplayTypeCombo * display_ctl = Q_NULLPTR;
};

#endif /* __QJovianEllipseSettings_h__ */
