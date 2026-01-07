/*
 * QLCSCTPCameraControls.h
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLCSCTPCameraControls_h__
#define __QLCSCTPCameraControls_h__

#include <gui/widgets/UpdateControls.h>
#include "QCameraControlsWidget.h"
#include "QLCSCTPUrlWidget.h"
#include "QLCSCTPCamera.h"

namespace serimager {

class QLCSCTPCameraControls :
    public QCameraControlsWidget
{
  Q_OBJECT;
public:
  typedef QLCSCTPCameraControls ThisClass;
  typedef QCameraControlsWidget Base;

  QLCSCTPCameraControls(const QLCSCTPCamera::sptr & camera, QWidget * parent = nullptr);
  ~QLCSCTPCameraControls();

protected :
  void onupdatecontrols() override;
  void populateCameras();
  void populateCameraControls();
  void populateStreams();
  void populateFormats();
  void populateSizes();

protected Q_SLOTS:
  void onCameraStateChanged();

protected:
  QLCSCTPCamera::sptr _camera;
  QLCSCTPUrlWidget * url_ctl = nullptr;
  QComboBox * cameras_ctl = nullptr;
  //  QLabel * model_ctl = nullptr;
  QComboBox * streams_ctl = nullptr;
  QComboBox * formats_ctl = nullptr;
  QComboBox * sizes_ctl = nullptr;
  QSpinBox* cameraDeviceBuffers_ctl = nullptr;
  QList<QLineEditBox*> cameraControls;

};

} /* namespace serimager */

#endif /* __QLCSCTPCameraControls_h__ */
