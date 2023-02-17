/*
 * QJovianEllipseDetectorSettings.h
 *
 *  Created on: Aug 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QJovianEllipseDetectorSettings_h__
#define __QJovianEllipseDetectorSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/proc/image_registration/c_jovian_derotation.h>

class QJovianEllipseDetectorSettings:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QJovianEllipseDetectorSettings ThisClass;
  typedef QSettingsWidget Base;

  QJovianEllipseDetectorSettings(QWidget * parent = nullptr);

  void set_jovian_ellipse_detector_options(c_jovian_ellipse_detector_options * options);
  c_jovian_ellipse_detector_options * jovian_ellipse_detector_options() const;


public Q_SLOTS:
  void copyParametersToClipboard();
  void pasteParametersFromClipboard();

protected:
  void onupdatecontrols() override;

protected:
  c_jovian_ellipse_detector_options * options_ = nullptr;
  QNumberEditBox * stdev_factor_ctl = nullptr;
  QCheckBox * force_reference_ellipse_ctl = nullptr;
  QToolBar * toolbar_ctl = nullptr;
};

#endif /* __QJovianEllipseDetectorSettings_h__ */
