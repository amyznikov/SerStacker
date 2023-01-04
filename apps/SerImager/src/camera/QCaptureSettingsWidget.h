/*
 * QCaptureSettingsWidget.h
 *
 *  Created on: Dec 29, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCaptureSettingsWidget_h__
#define __QCaptureSettingsWidget_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/UpdateControls.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include "QCameraWriter.h"

namespace serimager {

class QCameraCaptureControl:
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QCameraCaptureControl ThisClass;
  typedef QWidget Base;

  QCameraCaptureControl(QWidget * parent = nullptr);

  void setCameraWriter(QCameraWriter * writer);
  QCameraWriter * cameraWriter() const;

protected:
  void populateCaptureLimitsCombobox();
  bool getSelectedCaptureLimits(c_capture_limits * c);
  void onupdatecontrols() override;

protected Q_SLOTS:
  void onUpdateControls();
  void onStartStopButtonClicked();
  void onLimitsSelectionChanged(int);

protected:
  QCameraWriter *writer_ = nullptr;
  QVBoxLayout * vbox_ = nullptr;

  QHBoxLayout *h1_ = nullptr;
  QComboBox *limitsSelection_ctl = nullptr;
  QToolButton *startStop_ctl = nullptr;
};

class QCaptureSettingsWidget:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QCaptureSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QCaptureSettingsWidget(QWidget * parent = nullptr);
  ~QCaptureSettingsWidget();

  void setCameraWriter(QCameraWriter * writer);
  QCameraWriter * cameraWriter() const;

protected:
  void onupdatecontrols() override;


protected:
  QCameraWriter *writer_ = nullptr;
  QCameraCaptureControl * capture_ctl = nullptr;
  QSpinBox *num_rounds_ctl = nullptr;
  QSpinBox *interval_between_rounds_ctl = nullptr;
  QBrowsePathCombo * outpuPath_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QCaptureSettingsWidget_h__ */
