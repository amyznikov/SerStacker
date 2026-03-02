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

  QString name() const final;
  QString parameters() const final;

  bool is_same_camera(const QImagingCamera::sptr & rhs) const final;
  int drops() const final;
  // bool check_status() final;

  static QList<QImagingCamera::sptr> detectCameras();

  const ASI_CAMERA_INFO & cameraInfo() const;

protected:
  bool device_is_connected() const final;
  bool device_connect() final;
  void device_disconnect() final;
  bool device_start() final;
  void device_stop() final;
  int device_max_qsize() final;
  void device_release_frame(const QCameraFrame::sptr & frame) final;
  bool device_recv_frame(QCameraFrame::sptr & frm) final;
  void onload(const QSettings & settings, const QString & prefix) final;
  void onsave(QSettings & settings, const QString & prefix) final;

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
