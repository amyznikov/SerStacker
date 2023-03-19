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

/**
 * Provides Combobox for capture limits selection
 * and start / stop capture buttons
 * */
class QCaptureLimitsControl:
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QCaptureLimitsControl ThisClass;
  typedef QWidget Base;

  QCaptureLimitsControl(QWidget * parent = nullptr);

  void setCameraWriter(QCameraWriter * writer);
  QCameraWriter * cameraWriter() const;

Q_SIGNALS:
  void captureLimitChanged();

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
  QShortcut * startStopSortuct_ = nullptr;
};


class QStereoStreamCaptureOptions :
    public QSettingsWidget
{
public:
  typedef QStereoStreamCaptureOptions ThisClass;
  typedef QSettingsWidget Base;

  QStereoStreamCaptureOptions(QWidget * parent = nullptr);

  void setCameraWriter(QCameraWriter * writer);
  QCameraWriter * cameraWriter() const;

protected:
  void onupdatecontrols() override;

protected:
  QCameraWriter *writer_ = nullptr;
  QCheckBox * enable_split_stereo_stream_ctl = nullptr;
  QEnumComboBox<stereo_stream_layout_type> * stereo_stream_layout_type_ctl = nullptr;
  QCheckBox * enable_swap_cameras_ctl = nullptr;
  QCheckBox * downscale_panes_ctl = nullptr;
};



class QCaptureSettingsWidget:
    public QSettingsWidget
{
public:
  typedef QCaptureSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QCaptureSettingsWidget(QWidget * parent = nullptr);

  void setCameraWriter(QCameraWriter * writer);
  QCameraWriter * cameraWriter() const;

protected:
  void onupdatecontrols() override;
  void onload(QSettings & settings) override;
  void saveCaptureLimits();
  void loadCaptureLimits(QSettings & settings);
  void onAviOptionsMenuButtonClicked();


protected:
  QCameraWriter *writer_ = nullptr;
  QCaptureLimitsControl * captureLimits_ctl = nullptr;
  QSpinBox *num_rounds_ctl = nullptr;
  QSpinBox *interval_between_rounds_ctl = nullptr;
  QBrowsePathCombo * outpuPath_ctl = nullptr;
  QEnumComboBox<QCameraWriter::FORMAT> * output_format_ctl = nullptr;
  QLineEditBox * avi_options_ctl = nullptr;
  QToolButton * avi_options_menubutton_ctl = nullptr;
  QStereoStreamCaptureOptions * stereo_stream_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QCaptureSettingsWidget_h__ */
