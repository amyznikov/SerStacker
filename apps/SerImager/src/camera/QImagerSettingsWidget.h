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

class QImagerSettingsWidget:
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QImagerSettingsWidget ThisClass;
  typedef QFrame Base;

  QImagerSettingsWidget(QWidget * parent = nullptr);

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


class QCameraControlDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QCameraControlDock ThisClass;
  typedef QCustomDockWidget Base;

  QCameraControlDock(const QString & title, QWidget * parent, QImagerSettingsWidget * view);

protected:
};

} /* namespace qserimager */

#endif /* __QImagerSettingsWidget_h__ */
