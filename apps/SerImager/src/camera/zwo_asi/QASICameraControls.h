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


namespace serimager {

class QASIControlWidget :
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;

public:
  typedef QASIControlWidget ThisClass;
  typedef QWidget Base;

  QASIControlWidget(int iCameraID, const ASI_CONTROL_CAPS & ControlCaps,
      QWidget * parent = nullptr );

protected:
  void onupdatecontrols() override;
  void setASIControlValue(long lValue, ASI_BOOL isAuto);
  bool getASIControlValue(long * lValue, ASI_BOOL * isAuto);
  void updateCtrlRanges();

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
  public QWidget,
  public HasUpdateControls
{
  Q_OBJECT;

public:
  typedef QASIROIControlWidget ThisClass;
  typedef QWidget Base;

  QASIROIControlWidget(const QASICamera::sptr & camera,
      QWidget * parent = nullptr );

protected:
  void populate_supported_image_formats();
  void populate_supported_bins();
  void populate_available_frame_sizes(int iBin = 0);
  void onupdatecontrols() override;

  //void update_image_format_ctl(ASI_IMG_TYPE iFormat);
  //void add_available_frame_size(const QRect & rc);

  //void setBinningCtl(int iBin);

protected Q_SLOTS:
  void onUpdateControls();
  void onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState);
  void setCameraROI();

protected:
  QASICamera::sptr camera_;
  QHBoxLayout * layout_ = nullptr;
  QComboBox * frameSize_ctl = nullptr;
  QComboBox * imageFormat_ctl = nullptr;
  QComboBox * binning_ctl = nullptr;
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
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected Q_SLOTS:
  void onCameraStateChanged();

protected:
  QASICamera::sptr camera_;
  QWidgetList controls_;
};

class QASICameraControls:
    public QCameraControlsWidget
{
  Q_OBJECT;
public:
  typedef QASICameraControls ThisClass;
  typedef QCameraControlsWidget Base;

  QASICameraControls(const QASICamera::sptr & camera, QWidget * parent = nullptr);
  ~QASICameraControls();

protected:
  void create_controls();
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;

protected Q_SLOTS:
  void onCameraStateChanged();

protected:
  QASICamera::sptr camera_;
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
