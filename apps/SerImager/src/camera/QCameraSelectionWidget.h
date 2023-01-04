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

Q_SIGNALS:
  void selectedCameraChanged();

protected Q_SLOTS:
  void onCameraSelectionCurrentIndexChanged(int);
  void onConnectionStatusCtrlClicked();
  void onStartStopCtrlClicked();
  void onCameraInfoCtrlClicked();
  void onUpdateControls();

protected:
  void onupdatecontrols() override;
  void timerEvent(QTimerEvent *event) override;

protected:
  QImagingCamera::sptr getSelectedCamera() const;
  void refreshCameras();

protected:
  int refreshCamerasTimerId_ = -1;
  QImagingCamera::sptr selectedCamera_;

  QHBoxLayout * layout_ = nullptr;
  QComboBox * cameraSelection_ctl = nullptr;
  QToolButton * connectionStatus_ctl = nullptr;
  QToolButton * startStop_ctl = nullptr;
  QToolButton * cameraInfo_ctl = nullptr;
};

} /* namespace qserimager */

#endif /* __QCameraSelectionWidget_h__ */
