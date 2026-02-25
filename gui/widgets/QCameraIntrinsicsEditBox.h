/*
 * QCameraIntrinsicsEditBox.h
 *
 *  Created on: Aug 2, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraIntrinsicsEditBox_h__
#define __QCameraIntrinsicsEditBox_h__

#include <gui/widgets/QSettingsWidget.h>
//#include <gui/widgets/QSettingsWidgetTemplate.h>
#include <gui/widgets/QMatrixEdit.h>
#include <core/proc/camera_calibration/camera_calibration.h>

class QCameraIntrinsicsEditBox :
    public QSettingsWidget // Template2<c_camera_intrinsics>
{
public:
  typedef QCameraIntrinsicsEditBox ThisClass;
  typedef QSettingsWidget Base;

  QCameraIntrinsicsEditBox(QWidget * parent = nullptr);

  void set_camera_intrinsics(c_camera_intrinsics * camera_intrinsics);
  c_camera_intrinsics * camera_intrinsics() const;

protected:
  void showOptionsMenu();

protected:
  c_camera_intrinsics * _camera_intrinsics = nullptr;
  QNumericBox * imageSize_ctl = nullptr;
  QMatrixEdit * cameraMatrix_ctl = nullptr;
  QNumericBox * distCoeffs_ctl = nullptr;
  QToolButton * options_ctl = nullptr;
};

#endif /* __QCameraIntrinsicsEditBox_h__ */
