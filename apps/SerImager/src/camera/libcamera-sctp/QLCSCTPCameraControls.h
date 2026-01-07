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
  void populateStreams();
  void populateFormats();
  void populateSizes();
  void populateCameraControls();
  void updateBasicSensorControls();
  void updateColorToneControls();
  void updateVisualAdjustmentsControls();

  QCheckBox * createCheckBoxCameraControl(QSettingsWidget * sw, const QString & ctlid, const QString & name, const QString & tooltip);
  void updateCameraControl(QCheckBox * cb, const QLCSCTPCamera::QLCCamera * cam);

  QSpinBox * createSpinBoxCameraControl(QSettingsWidget * sw, const QString & ctlid, const QString & name, const QString & tooltip);
  void updateCameraControl(QSpinBox * sb, const QLCSCTPCamera::QLCCamera * cam);

  QDoubleSpinBox * createDoubleSpinBoxCameraControl(QSettingsWidget * sw, const QString & ctlid, const QString & name, const QString & tooltip);
  void updateCameraControl(QDoubleSpinBox * sb, const QLCSCTPCamera::QLCCamera * cam);

  QComboBox * createComboBoxCameraControl(QSettingsWidget * sw, const QString & ctlid, const QString & name, const QString & tooltip);
  void updateCameraControl(QComboBox * cb, const QLCSCTPCamera::QLCCamera * cam);


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

  // Exposure & Sensor Hardware Controls
  QCheckBox * AeEnable_ctl = nullptr;
  QSpinBox * ExposureTime_ctl = nullptr;
  QDoubleSpinBox * AnalogueGain_ctl = nullptr;
  QComboBox * AeExposureMode_ctl = nullptr;
  QComboBox * AeMeteringMode_ctl = nullptr;
  QComboBox * AeConstraintMode_ctl = nullptr;
  QComboBox * AeFlickerMode_ctl = nullptr;
  QSpinBox * AeFlickerPeriod_ctl = nullptr;

  // ISP Color & White Balance
  QExpandableGroupBox * ISPColorToneGroup_ctl = nullptr;
  QSettingsWidget * ISPColorToneControls_ctl = nullptr;
  QCheckBox * AwbEnable_ctl = nullptr;
  QComboBox * AwbMode_ctl = nullptr;
  QSpinBox * ColourTemperature_ctl = nullptr;
//  QNumericBox * ColourGains_ctl = nullptr;
//  QNumericBox * ColourCorrectionMatrix_ctl = nullptr;

  // ISP Visual Adjustments
  QExpandableGroupBox * ISPVisualAdjustmentsGroup_ctl = nullptr;
  QSettingsWidget * ISPVisualAdjustmentsControls_ctl = nullptr;
  QDoubleSpinBox * Brightness_ctl = nullptr;
  QDoubleSpinBox * Contrast_ctl = nullptr;
  QDoubleSpinBox * Saturation_ctl = nullptr;
  QDoubleSpinBox * Gamma_ctl = nullptr;
  QDoubleSpinBox * Sharpness_ctl = nullptr;
  QComboBox * NoiseReductionMode_ctl = nullptr;
  QComboBox * HdrMode_ctl = nullptr;

  //QDoubleSpinBox * LensPosition_ctl = nullptr;

};

} /* namespace serimager */

#endif /* __QLCSCTPCameraControls_h__ */
