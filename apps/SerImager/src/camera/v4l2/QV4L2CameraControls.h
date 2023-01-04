/*
 * QV4L2CameraControls.h
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QV4L2CameraControls_h__
#define __QV4L2CameraControls_h__

#include <gui/widgets/UpdateControls.h>
#include "QCameraControlsWidget.h"
#include "QV4L2Camera.h"

namespace serimager {

class QV4L2CameraExtraSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QV4L2CameraExtraSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QV4L2CameraExtraSettingsWidget(const QV4L2Camera::sptr & camera,
      QWidget * parent = nullptr);
  ~QV4L2CameraExtraSettingsWidget();

protected:
  QWidget* add_ex_ctrl(cv4l_fd & device, const v4l2_query_ext_ctrl & c);
  void createControls();
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;


protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldSate,
      QImagingCamera::State newState);

protected:
  QV4L2Camera::sptr camera_;
  QWidgetList controls_;
};


class QV4L2CameraControls:
    public QCameraControlsWidget
{
  Q_OBJECT;
public:
  typedef QV4L2CameraControls ThisClass;
  typedef QCameraControlsWidget Base;

  QV4L2CameraControls(const QV4L2Camera::sptr & camera, QWidget * parent  = nullptr);
  ~QV4L2CameraControls();

protected:
  void createControls();
  void deleteControls();
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;
  void refreshFormats(cv4l_fd & device);
  void refreshFrameSizes(cv4l_fd & device);
  void refreshFrameRates(cv4l_fd & device);

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldSate, QImagingCamera::State newState);
  void onVideoInputChanged(int index);
  void onInputFormatChanged(int index);
  void onFrameSizeChanged(int index);
  void onFrameWidthChanged(int value);
  void onFrameHeightChanged(int value);
  void onFrameRateChanged(int index);


protected:
  QV4L2Camera::sptr camera_;

  QComboBox * videoInput_ctl = nullptr;
  QComboBox *fmt_ctl = nullptr;
  QComboBox *frameSize_ctl = nullptr;
  QSpinBox *frameWidth_ctl = nullptr;
  QSpinBox *frameHeight_ctl = nullptr;
  QComboBox *frameRate_ctl = nullptr;
  QV4L2CameraExtraSettingsWidget * extraControls_ctl = nullptr;

};

} /* namespace serimager */

#endif /* __QV4L2CameraControls_h__ */
