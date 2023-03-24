/*
 * QImagerSettingsWidget.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImagerSettingsWidget_h__
#define __QImagerSettingsWidget_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/UpdateControls.h>
#include <gui/qcustomdock/QCustomDock.h>
#include "QCameraSelectionWidget.h"
#include "QCameraControlsWidget.h"
#include "QCaptureSettingsWidget.h"
#include "QCameraWriter.h"

namespace serimager {

class QImagingCameraControlsWidget:
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QImagingCameraControlsWidget ThisClass;
  typedef QFrame Base;

  QImagingCameraControlsWidget(QWidget * parent = nullptr);

  const QImagingCamera::sptr &selectedCamera() const;

  void setCameraWriter(QCameraWriter * writer);
  QCameraWriter * cameraWriter() const;

Q_SIGNALS:
  void selectedCameraChanged();

protected Q_SLOTS:
  void onSelectedCameraChanged();

protected:
  void onupdatecontrols() override;

protected:
  QVBoxLayout * layout_ = nullptr;
  QCameraSelectionWidget * cameraSelection_ctl = nullptr;
  QSettingsWidget * settings_ctl = nullptr;

  QCameraControlsWidget * cameraControls_ctl = nullptr;
  QCaptureSettingsWidget * captureSettings_ctl = nullptr;
};


class QImagingCameraControlsDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QImagingCameraControlsDock ThisClass;
  typedef QCustomDockWidget Base;

  QImagingCameraControlsDock(const QString & title, QWidget * parent,
      QImagingCameraControlsWidget * view);

protected:
};

} /* namespace serimager */

#endif /* __QImagerSettingsWidget_h__ */
