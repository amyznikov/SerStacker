/*
 * QCameraSelectionWidget.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraSelectionWidget_h__
#define __QCameraSelectionWidget_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/UpdateControls.h>
#include "QImagingCamera.h"

namespace serimager {

class QCameraSelectionWidget:
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QCameraSelectionWidget ThisClass;
  typedef QFrame Base;

  QCameraSelectionWidget(QWidget * parent = nullptr);

  const QImagingCamera::sptr & selectedCamera() const;

  void loadSettings(const QString & prefix = "");
  void loadSettings(const QSettings & settings, const QString & prefix="");
  void saveSettings(const QString & prefix="");
  void saveSettings(QSettings & settings, const QString & prefix="");

Q_SIGNALS:
  void selectedCameraChanged();

protected Q_SLOTS:
  void onCameraSelectionCurrentIndexChanged(int);
  void onConnectionStatusCtrlClicked();
  void onStartStopCtrlClicked();
  void onCameraInfoCtrlClicked();
  void onMenuCtrlClicked();
  void onUpdateControls();

protected:
  void onupdatecontrols() override;
  void timerEvent(QTimerEvent *event) override;

protected:
  QImagingCamera::sptr getSelectedCamera();
  void refreshCameras();

protected:
  QString _camerasPrefix = "CameraSelection/cameras";
  int _refreshCamerasTimerId = -1;
  QImagingCamera::sptr _selectedCamera;

  QHBoxLayout * _layout = nullptr;
  QComboBox * cameraSelection_ctl = nullptr;
  QToolButton * connectionStatus_ctl = nullptr;
  QToolButton * startStop_ctl = nullptr;
  //QToolButton * cameraInfo_ctl = nullptr;
  QToolButton * menu_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QCameraSelectionWidget_h__ */
