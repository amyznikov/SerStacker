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

namespace qserstacker {

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
  QEnumComboBox<DEBAYER_ALGORITHM> * editor_debayer_ctl = nullptr;
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

protected:
  QGeneralAppSettingsWidget * appSettingsWidget_ = nullptr;
};

} // namespace qserstacker
#endif /* __QAppSettings_h__ */
