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

class QGeneralAppSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QGeneralAppSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QGeneralAppSettingsWidget(QWidget * parent = Q_NULLPTR);

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = Q_NULLPTR;
};


class QGeneralAppSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QGeneralAppSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QGeneralAppSettingsDialogBox(QWidget * parent = Q_NULLPTR);

protected:
protected:
  QGeneralAppSettingsWidget * appSettingsWidget_ = Q_NULLPTR;
};


#endif /* __QAppSettings_h__ */
