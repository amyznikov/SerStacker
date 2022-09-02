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
#include <core/proc/jupiter.h>

class QJovianEllipseDetectorSettings:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QJovianEllipseDetectorSettings ThisClass;
  typedef QSettingsWidget Base;

  QJovianEllipseDetectorSettings(QWidget * parent = Q_NULLPTR);

  void set_jovian_ellipse_detector_options(c_jovian_ellipse_detector_options * options);
  c_jovian_ellipse_detector_options * jovian_ellipse_detector_options() const;


public slots:
  void copyParametersToClipboard();
  void pasteParametersFromClipboard();

protected:
  void onupdatecontrols() override;

protected:
  c_jovian_ellipse_detector_options * options_ = Q_NULLPTR;
  QNumberEditBox * hlines_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_scale_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_blur_ctl = Q_NULLPTR;
  QNumberEditBox * gradient_blur_ctl = Q_NULLPTR;
  QToolBar * toolbar_ctl = Q_NULLPTR;
};

#endif /* __QJovianEllipseDetectorSettings_h__ */
