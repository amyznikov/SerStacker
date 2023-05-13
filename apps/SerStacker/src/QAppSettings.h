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
#include "QImageEditor.h"

namespace serstacker {

class QGeneralAppSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QGeneralAppSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QGeneralAppSettingsWidget(QWidget * parent = nullptr);

  void setImageEditor(QImageEditor * imageEditor);
  QImageEditor * imageEditor() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QImageEditor * imageEditor_ = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * editorDebayer_ctl = nullptr;
  QCheckBox * dropBadPixels_ctl = nullptr;
  QNumericBox * badPixelsVariationThreshold_ctl = nullptr;
};


class QGeneralAppSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QGeneralAppSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGeneralAppSettingsDialogBox(QWidget * parent = nullptr);

  void setImageEditor(QImageEditor * imageEditor);
  QImageEditor * imageEditor() const;

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
