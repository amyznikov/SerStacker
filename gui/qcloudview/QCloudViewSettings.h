/*
 * QCloudViewSettings.h
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewSettings_h__
#define __QCloudViewSettings_h__

#include "QCloudViewer.h"
#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QColorPickerButton.h>


class QCloudViewSettings :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewSettings ThisClass;
  typedef QSettingsWidget Base;

  QCloudViewSettings(QWidget * parent = nullptr);

  void setCloudViewer(QCloudViewer * v);
  QCloudViewer * cloudViewer() const;

  void refreshCloudList();


protected:
  //void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QCloudViewer * cloudViewer_ = nullptr;
  QEnumComboBox<QGLView::Projection> * projection_ctl = nullptr;
  QNumericBox * nearPlane_ctl = nullptr;
  QNumericBox * farPlane_ctl = nullptr;
  QNumericBox * fov_ctl = nullptr;
  QNumericBox * mainAxesLength_ctl = nullptr;
  QNumericBox * sceneTarget_ctl = nullptr;
  QNumericBox * upDirection_ctl = nullptr;
  QNumericBox * sceneOrigin_ctl = nullptr;
  QCheckBox   * autoShowViewTarget_ctl = nullptr;
  QNumericBox * pointSize_ctl = nullptr;
  QNumericBox * pointBrightness_ctl = nullptr;
  QColorPickerButton * bgColor_ctl = nullptr;
};

class QCloudViewSettingsWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewSettingsWidget ThisClass;
  typedef QWidget Base;

  QCloudViewSettingsWidget(QWidget * parent = nullptr);

  void setCloudViewer(QCloudViewer * v);
  QCloudViewer * cloudViewer() const;

protected:
  QCloudViewer * cloudViewer_ = nullptr;
  QCloudViewSettings * settings_ctl = nullptr;
  QToolButton * rotateCameraToShowCloud_ctl = nullptr;
  QToolButton * moveCameraToShowCloud_ctl = nullptr;
  QToolButton * showKeyBindings_ctl = nullptr;
};

class QCloudViewSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QCloudViewSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QCloudViewSettingsDialogBox(QWidget * parent = nullptr);

  void setCloudViewer(QCloudViewer * v);
  QCloudViewer * cloudViewer() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QVBoxLayout * vbox_ = nullptr;
  QCloudViewSettingsWidget * cloudViewSettingsWidget_ = nullptr;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;

};

#endif /* __QCloudViewSettings_h__ */
