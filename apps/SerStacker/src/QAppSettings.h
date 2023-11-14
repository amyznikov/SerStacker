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

class QGeneralAppSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QGeneralAppSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QGeneralAppSettingsWidget(QWidget * parent = nullptr);

  void setInputSequenceView(QInputSequenceView * sequenceView);
  QInputSequenceView * inputSequenceView() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QInputSequenceView *  sequenceView_ = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * editorDebayer_ctl = nullptr;
  QCheckBox * dropBadPixels_ctl = nullptr;
  QNumericBox * badPixelsVariationThreshold_ctl = nullptr;
  QEnumComboBox<c_input_source::OUTPUT_TYPE> * sourceOutputType_ctl_ = nullptr;

#if HAVE_VLO_FILE
  QEnumComboBox<c_vlo_file::DATA_CHANNEL> * vloDataChannel_ctl_ = nullptr;
#endif
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
