/*
 * QCameraMatrixSelectionDialogBox.h
 *
 *  Created on: Oct 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraMatrixSelectionDialogBox_h__
#define __QCameraMatrixSelectionDialogBox_h__

#include <QtWidgets/QtWidgets>
#include <core/settings/camera_settings.h>

class QCameraMatrixDialogBox :
    public QDialog
{
  // Q_OBJECT;
public:
  typedef QCameraMatrixDialogBox ThisClass;
  typedef QDialog Base;

  struct Camera {
    std::string name;
    c_camera_intrinsics intrinsics;
  };

  QCameraMatrixDialogBox(QWidget * parent = nullptr);


protected:
  void load_cameras();
  void save_cameras();

protected:
  std::vector<Camera> cameras_;

  QLineEdit * filter_ctl = nullptr;
  QListWidget * list_ctl = nullptr;
  QPushButton * btnOK = nullptr;
  QPushButton * btnCancel = nullptr;
  int selectedIndex_ = -1;
};

#endif /* __QCameraMatrixSelectionDialogBox_h__ */
