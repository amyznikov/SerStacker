/*
 * QASICamera.h
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QASICamera_h__
#define __QASICamera_h__

#include "QImagingCamera.h"
#ifdef _WIN32
# include "zwo_asi/ASI_Windows_SDK_V1.27/include/ASICamera2.h"
#else
# include "zwo_asi/ASI_linux_mac_SDK_V1.27/include/ASICamera2.h"
#endif


namespace serimager {

class QASICamera :
    public QImagingCamera
{
public:
  typedef QASICamera ThisClass;
  typedef QImagingCamera Base;
  typedef std::shared_ptr<ThisClass> sptr;

  QASICamera(const ASI_CAMERA_INFO & camInfo,
      QObject * parent = nullptr);

  ~QASICamera();

  static sptr create(const ASI_CAMERA_INFO & camInfo,
      QObject * parent = nullptr);

  QString display_name() const override;
  QString parameters() const override;

  bool is_same_camera(const QImagingCamera::sptr & rhs) const override;
  int drops() const override;
  // bool check_status() override;

  static QList<QImagingCamera::sptr> detectCameras();

  const ASI_CAMERA_INFO & cameraInfo() const;

protected:
  bool device_is_connected() const override;
  bool device_connect() override;
  void device_disconnect() override;
  bool device_start() override;
  void device_stop() override;
  int device_max_qsize() override;
  void device_release_frame(const QCameraFrame::sptr & frame) override;
  bool device_recv_frame(QCameraFrame::sptr & frm) override;

protected:
  bool create_frame_buffers(const cv::Size & imageSize,
      int cvType,
      enum COLORID colorid,
      int bpp,
      int num_buffers);

  void asi_close();
  void qpool(const QCameraFrame::sptr & );
  QCameraFrame::sptr dqpool();

protected:
  ASI_CAMERA_INFO _camInfo;
  std::vector<QCameraFrame::sptr> _p;
  bool _is_asi_open = false;
};

} /* namespace serimager */

#endif /* __QASICamera_h__ */
