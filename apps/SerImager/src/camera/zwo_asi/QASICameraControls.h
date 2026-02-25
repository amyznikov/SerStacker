/*
 * QASICameraControls.h
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QASICameraControls_h__
#define __QASICameraControls_h__

#include <gui/widgets/UpdateControls.h>
#include "QCameraControlsWidget.h"
#include "QASICamera.h"
#include "QCameraROISelectionDialog.h"


namespace serimager {

class QASIControlWidgetBase:
    public QWidget,
    public HasUpdateControls
{
public:
  typedef QASIControlWidgetBase ThisClass;
  typedef QWidget Base;

protected:
  QASIControlWidgetBase(QWidget * parent = nullptr) :
      Base(parent)
  {
  }
};

class QASIControlWidget :
    public QASIControlWidgetBase
{
  Q_OBJECT;

public:
  typedef QASIControlWidget ThisClass;
  typedef QASIControlWidgetBase Base;

  QASIControlWidget(int iCameraID, const ASI_CONTROL_CAPS & ControlCaps,
      QWidget * parent = nullptr );

protected:
  void onupdatecontrols() override;
  void setASIControlValue(long lValue, ASI_BOOL isAuto);
  bool getASIControlValue(long * lValue, ASI_BOOL * isAuto);
  void updateCtrlRange();

protected Q_SLOTS:
  void onValueCtlChanged(int);
  void onSliderCtlChanged(int);
  void onAutoCtlStateChanged(int);
  void onExpScaleCtlChanged(int);


protected:
  ASI_CONTROL_CAPS controlCaps_;
  int iCameraID;
  QVBoxLayout * lv_ = nullptr;
  QHBoxLayout * lh_ = nullptr;
  QSpinBox * value_ctl = nullptr;
  QComboBox * expscale_ctl = nullptr;
  QCheckBox * auto_ctl = nullptr;
  QSlider * slider_ctl = nullptr;
};

class QASIROIControlWidget :
    public QASIControlWidgetBase
{
  Q_OBJECT;

public:
  typedef QASIROIControlWidget ThisClass;
  typedef QASIControlWidgetBase Base;

  QASIROIControlWidget(const QASICamera::sptr & camera,
      QWidget * parent = nullptr );

  ~QASIROIControlWidget();

protected:
  void onupdatecontrols() override;
  void populate_supported_frame_formats();
  void update_supported_frame_sizes_combobox();
  bool set_capture_format(int iX, int iY, int iWidth, int iHeight,  int iBin, ASI_IMG_TYPE iFormat);
  bool get_capture_format(int * iX, int * iY, int * iWidth, int * iHeight, int * iBin, ASI_IMG_TYPE * iFormat);
  bool setup_camera_capture_format();

protected Q_SLOTS:
  void onUpdateControls();
  void onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState);
  void onFrameSizeCtlCurrentIndexChanged();
//  void onImageFormatCtlCurrentIndexChanged();
//  void onBinningCtlCurrentIndexChanged();

protected:
  QASICamera::sptr camera_;
  QHBoxLayout * layout_ = nullptr;
  QComboBox * frameSize_ctl = nullptr;
  QComboBox * imageFormat_ctl = nullptr;
  QComboBox * binning_ctl = nullptr;

  QList<QCameraROI> predefinedROIs_;
  QList<QCameraROI> userDefinedROIs_;

  QImagingCamera::PreStartProc prestartproc_;
};

class QASICameraExtraContolsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QASICameraExtraContolsWidget ThisClass;
  typedef QSettingsWidget Base;

  QASICameraExtraContolsWidget(const QASICamera::sptr & camera,
      QWidget * parent = nullptr);

  ~QASICameraExtraContolsWidget();

protected:
  void createControls();

protected Q_SLOTS:
  void onCameraStateChanged();

protected:
  QASICamera::sptr _camera;
  QList<QASIControlWidgetBase*> _controls;

};

class QASICameraControls:
    public QCameraControlsWidget
{
  Q_OBJECT;
public:
  typedef QASICameraControls ThisClass;
  typedef QCameraControlsWidget Base;

  QASICameraControls(const QASICamera::sptr & camera,
      QWidget * parent = nullptr);

  ~QASICameraControls();

protected:
  void create_controls();

protected Q_SLOTS:
  void onCameraStateChanged();

protected:
  QASICamera::sptr _camera;
  QASIROIControlWidget * roi_ctl = nullptr;
  QASIControlWidget * exposure_ctl = nullptr;
  QASIControlWidget * gain_ctl = nullptr;
  QASICameraExtraContolsWidget * extraSettings_ctl = nullptr;
};

struct QASIExposureScaleParams
{
  long minValue = 0;
  long maxValue = 0;
  long scale = 0;

  QASIExposureScaleParams()
  {
  }

  QASIExposureScaleParams(long _min, long _max, long _scale) :
      minValue(_min), maxValue(_max), scale(_scale)
  {
  }
};

} /* namespace serimager */

Q_DECLARE_METATYPE(serimager::QASIExposureScaleParams)

#endif /* __QASICameraControls_h__ */
