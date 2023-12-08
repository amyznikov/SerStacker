/*
 * QAppSettings.h
 *
 *  Created on: Nov 6, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QAppSettings_h__
#define __QAppSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/io/debayer.h>
#include <gui/qinputsequenceview/QInputSequenceView.h>

namespace serstacker {

class QGeneralAppInputSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QGeneralAppInputSettings ThisClass;
  typedef QSettingsWidget Base;

  QGeneralAppInputSettings(QWidget * parent = nullptr);

  void setInputSequenceView(QInputSequenceView * sequenceView);
  QInputSequenceView * inputSequenceView() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QInputSequenceView * sequenceView_ = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * editorDebayer_ctl = nullptr;
  QCheckBox * dropBadPixels_ctl = nullptr;
  QNumericBox * badPixelsVariationThreshold_ctl = nullptr;
  QEnumComboBox<c_input_source::OUTPUT_TYPE> * sourceOutputType_ctl_ = nullptr;

};

class QVLOGhostFilterSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QVLOGhostFilterSettings ThisClass;
  typedef QSettingsWidget Base;

  QVLOGhostFilterSettings(QWidget * parent = nullptr);

  void setInputSequenceView(QInputSequenceView * sequenceView);
  QInputSequenceView * inputSequenceView() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QInputSequenceView *  sequenceView_ = nullptr;

#if HAVE_VLO_FILE
  QCheckBox * enableGhostFilter_ctl = nullptr;
  QNumericBox * saturation_level_ctl  = nullptr;
  QNumericBox * doubled_distanse_systematic_correction_ctl  = nullptr;
  QNumericBox * doubled_distanse_depth_tolerance_ctl  = nullptr;
#endif
};

class QVLOLowIntensityFilterSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QVLOLowIntensityFilterSettings ThisClass;
  typedef QSettingsWidget Base;

  QVLOLowIntensityFilterSettings(QWidget * parent = nullptr);

  void setInputSequenceView(QInputSequenceView * sequenceView);
  QInputSequenceView * inputSequenceView() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QInputSequenceView *  sequenceView_ = nullptr;

#if HAVE_VLO_FILE
  QCheckBox * enabled_ctl = nullptr;
  QNumericBox * low_intensity_level_ctl  = nullptr;
  QNumericBox * u_ctl = nullptr;
  QNumericBox * v_ctl = nullptr;
#endif
};


class QVLOInputSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QVLOInputSettings ThisClass;
  typedef QSettingsWidget Base;

  QVLOInputSettings(QWidget * parent = nullptr);

  void setInputSequenceView(QInputSequenceView * sequenceView);
  QInputSequenceView * inputSequenceView() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QInputSequenceView * sequenceView_ = nullptr;

#if HAVE_VLO_FILE
  QEnumComboBox<c_vlo_file::DATA_CHANNEL> * vloDataChannel_ctl_ = nullptr;
#endif

  QTabWidget * tab_ctl = nullptr;
  QVLOGhostFilterSettings * ghostFilter_ctl = nullptr;
  QVLOLowIntensityFilterSettings * lowIntensityFilter_ctl = nullptr;

};

class QGeneralAppSettingsWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QGeneralAppSettingsWidget ThisClass;
  typedef QWidget Base;

  QGeneralAppSettingsWidget(QWidget * parent = nullptr);

  void setInputSequenceView(QInputSequenceView * sequenceView);
  QInputSequenceView * inputSequenceView() const;

protected:
  QTabWidget * tab_ctl = nullptr;
  QGeneralAppInputSettings * genericInputSettings_ctl = nullptr;
  QVLOInputSettings * vloSettings_ctl = nullptr;
};


class QGeneralAppSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QGeneralAppSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGeneralAppSettingsDialogBox(QWidget * parent = nullptr);

  void setInputSequenceView(QInputSequenceView * sequenceView);
  QInputSequenceView * inputSequenceView() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QGeneralAppSettingsWidget * appSettingsWidget_ = nullptr;
};

} // namespace serstacker
#endif /* __QAppSettings_h__ */
