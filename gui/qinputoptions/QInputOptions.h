/*
 * QInputOptions.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputOptions_h__
#define __QInputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/io/c_input_options.h>
#include <core/io/hdl/c_hdl_specification.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

class QVideoInputOptions :
    public QSettingsWidget // Template<c_video_input_options>
{
  Q_OBJECT;
public:
  typedef QVideoInputOptions ThisClass;
  typedef QSettingsWidget Base; // Template<c_video_input_options> Base;

  QVideoInputOptions(QWidget * parent = nullptr);

  void set_video_input_options(c_video_input_options * options)
  {
    _options = options;
    updateControls();
  }

  c_video_input_options * video_input_options() const
  {
    return _options;
  }

protected:
  c_video_input_options * _options = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
  QCheckBox * enable_color_maxtrix_ctl = nullptr;
  QCheckBox * filter_bad_pixels_ctl = nullptr;
  QNumericBox * bad_pixels_variation_threshold_ctl = nullptr;
  //  QBrowsePathCombo * darkframe_ctl = nullptr;
  //  QNumericBox * darkFrameScale_ctl  = nullptr;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

class QHDLConfigOptions :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QHDLConfigOptions ThisClass;
  typedef QSettingsWidget Base;

  QHDLConfigOptions(QWidget * parent = nullptr);

protected:
  QEnumComboBox<HDLSensorType> * sensorType_ctl = nullptr;
  QBrowsePathCombo * configFilePathName_ctl = nullptr;
  // QNumericBox * nonlive_streams_delay_ctl = nullptr;
};


///////////////////////////////////////////////////////////////////////////////////////////////////

class QInputOptions :
    public QSettingsWidget // Template<c_input_options>
{
  Q_OBJECT;
public:
  typedef QInputOptions ThisClass;
  typedef QSettingsWidget Base; // Template<c_input_options> Base;

  QInputOptions(QWidget * parent = nullptr);

  void set_options(c_input_options * options);
  c_input_options * options() const;

protected:
  c_input_options * _options = nullptr;
  QTabWidget * tab_ctl = nullptr;
  QVideoInputOptions * videoOptions_ctl = nullptr;
  QHDLConfigOptions * hdlconfigOptions_ctl = nullptr;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

class QInputOptionsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QInputOptionsDialogBox ThisClass;
  typedef QDialog Base;

  QInputOptionsDialogBox(QWidget * parent = nullptr);

  void setInputOptions(c_input_options * options);
  c_input_options * inputOptions() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void parameterChanged();

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QInputOptions * inputOptions_ctl = nullptr;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

void loadHDLSensorTypeToConfigFileMapping();
void saveHDLSensorTypeToConfigFileMapping();

void loadHDLStreamsGlobalOptions();
void saveHDLStreamsGlobalOptions();

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif /* __QInputOptions_h__ */
