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
//#include "QPointCloudsSettingsControl.h"
#include <gui/widgets/QSettingsWidget.h>


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
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QCloudViewer * cloudViewer_ = nullptr;
  QNumberEditBox * nearPlane_ctl = nullptr;
  QNumberEditBox * farPlane_ctl = nullptr;
  QNumberEditBox * sceneTarget_ctl = nullptr;
  QNumberEditBox * upDirection_ctl = nullptr;
  QNumberEditBox * sceneOrigin_ctl = nullptr;
  QNumberEditBox * pointSize_ctl = nullptr;
  QNumberEditBox * pointBrightness_ctl = nullptr;

  //QPointCloudsSettingsControl * cloudsSettings_ctl = nullptr;

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

signals:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QVBoxLayout * vbox_ = nullptr;
  QCloudViewSettings * cloudViewSettings_ = nullptr;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;

};

#endif /* __QCloudViewSettings_h__ */
