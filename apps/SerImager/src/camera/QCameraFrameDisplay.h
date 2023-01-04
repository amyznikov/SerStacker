/*
 * QCameraFrameDisplay.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraFrameDisplay_h__
#define __QCameraFrameDisplay_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qimageview/QMtfImageDisplayFunction.h>
#include "QImagingCamera.h"

namespace serimager {

class QCameraFrameDisplaySettings :
    public QMtfImageDisplaySettings
{
  Q_OBJECT;
public:
  typedef QCameraFrameDisplaySettings ThisClass;
  typedef QMtfImageDisplaySettings Base;

  QCameraFrameDisplaySettings(QObject * parent = nullptr);
  const c_enum_member * displayTypes() const override;

  void loadParameters() override;
  void saveParameters() const override;
};


class QCameraFrameDisplay:
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QCameraFrameDisplay ThisClass;
  typedef QImageEditor Base;

  QCameraFrameDisplay(QWidget * parent = nullptr);
  ~QCameraFrameDisplay();

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr & camera() const;

  const QCameraFrameDisplaySettings * displaySettings() const;
  QCameraFrameDisplaySettings * displaySettings();


protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldSate,
      QImagingCamera::State newState);
  void onUpdateCameraFrameDisplay();

protected:
  void showCurrentDisplayImage() override;

protected:
  QImagingCamera::sptr camera_;
  QCameraFrameDisplaySettings displaySettings_;
  QMtfImageDisplayFunction displayFunction_;
  QTimer timer_;
  int last_index = -1;
};

} /* namespace qserimager */

#endif /* __QCameraFrameDisplay_h__ */
