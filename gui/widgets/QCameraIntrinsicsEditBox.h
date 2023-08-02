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
#include <gui/widgets/QMatrixEdit.h>
#include <core/proc/camera_calibration/camera_calibration.h>

class QCameraIntrinsicsEditBox :
    public QSettingsWidgetTemplate<c_camera_intrinsics>
{
public:
  typedef QCameraIntrinsicsEditBox ThisClass;
  typedef QSettingsWidgetTemplate<c_camera_intrinsics> Base;

  QCameraIntrinsicsEditBox(QWidget * parent = nullptr);

protected:
  void showOptionsMenu();

protected:
  QNumericBox * imageSize_ctl = nullptr;
  QMatrixEdit * cameraMatrix_ctl = nullptr;
  QNumericBox * distCoeffs_ctl = nullptr;
  QToolButton * options_ctl = nullptr;
};

#endif /* __QCameraIntrinsicsEditBox_h__ */
