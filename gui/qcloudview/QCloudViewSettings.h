/*
 * QCloudViewSettings.h
 *
 *  Created on: May 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewSettings_h__
#define __QCloudViewSettings_h__
#if HAVE_QGLViewer // Should come from CMakeLists.txt

#include "QCloudViewer.h"
//#include "QPointCloudsSettingsControl.h"
#include <gui/widgets/QSettingsWidget.h>


class QCloudViewSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewSettings ThisClass;
  typedef QSettingsWidget Base;

  QCloudViewSettings(QWidget * parent = Q_NULLPTR);

  void setCloudViewer(QCloudViewer * v);
  QCloudViewer * cloudViewer() const;

  void refreshCloudList();


protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QCloudViewer * cloudViewer_ = Q_NULLPTR;
  QNumberEditBox * sceneRadius_ctl = Q_NULLPTR;
  QNumberEditBox * sceneOrigin_ctl = Q_NULLPTR;
  QNumberEditBox * pointSize_ctl = Q_NULLPTR;
  QNumberEditBox * pointBrightness_ctl = Q_NULLPTR;
  QNumberEditBox * sceneCenter_ctl = Q_NULLPTR;

  //QPointCloudsSettingsControl * cloudsSettings_ctl = Q_NULLPTR;

};

class QCloudViewSettingsDialogBox
    : public QDialog
{
  Q_OBJECT;
public:
  typedef QCloudViewSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QCloudViewSettingsDialogBox(QWidget * parent = Q_NULLPTR);

  void setCloudViewer(QCloudViewer * v);
  QCloudViewer * cloudViewer() const;

signals:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
protected:
  QVBoxLayout * vbox_ = Q_NULLPTR;
  QCloudViewSettings * cloudViewSettings_ = Q_NULLPTR;
  QSize lastWidnowSize_;
  QPoint lastWidnowPos_;

};

#endif // HAVE_QGLViewer
#endif /* __QCloudViewSettings_h__ */
