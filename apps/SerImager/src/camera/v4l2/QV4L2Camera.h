/*
 * QV4L2Camera.h
 *
 *  Created on: Dec 22, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QV4L2Camera_h__
#define __QV4L2Camera_h__

#include "QImagingCamera.h"
#include "QV4L2CameraFrame.h"
#include <libv4lconvert.h>
#include <linux/v4l2-controls.h>

namespace serimager {

class QV4L2Camera :
    public QImagingCamera
{
public:
  typedef QV4L2Camera ThisClass;
  typedef QImagingCamera Base;
  typedef std::shared_ptr<ThisClass> sptr;

  QV4L2Camera(const QString & filename, QObject * parent = nullptr);
  ~QV4L2Camera();

  static sptr create(const QString & filename);

  QString name() const override;
  QString parameters() const override;

  bool is_same_camera(const QImagingCamera::sptr & rhs) const override;
  int drops() const override;

  static QList<QImagingCamera::sptr> detectCameras();

  cv4l_fd & device();
  const cv4l_fd & device() const;

  int g_ext_ctrl(v4l2_ext_control & c);
  int s_ext_ctrl(const v4l2_ext_control & c);

  int s_ext_ctrl(__u32 cid, __s32 value);
  int g_ext_ctrl(__u32 cid, __s32 * value);

  int s_ext_ctrl(__u32 cid, __s64 value);
  int g_ext_ctrl(__u32 cid, __s64 * value);

  int s_ext_ctrl(__u32 cid, bool value);
  int g_ext_ctrl(__u32 cid, bool * value);

  int s_ext_ctrl(__u32 cid, const QString & value);
  int g_ext_ctrl(__u32 cid, QString *value);

protected:
  bool device_is_connected() const override;
  bool device_connect() override;
  void device_disconnect() override;
  bool device_start() override;
  void device_stop() override;
  int device_max_qsize() override;
  void device_release_frame(const QCameraFrame::sptr & queue) override;
  bool device_recv_frame(QCameraFrame::sptr & frm) override;
  void onload(const QSettings & settings, const QString & prefix) final;
  void onsave(QSettings & settings, const QString & prefix) final;

  bool create_queue();
  bool dqbuf(cv4l_buffer & buf);

protected:
  QString _filename;
  cv4l_fd _device;
  cv4l_queue _q;
  std::vector<QV4L2CameraFrame::sptr> _p;
  struct v4lconvert_data *_convert = nullptr;
  cv4l_fmt srcFormat;
  cv4l_fmt dstFormat;
  struct v4l2_fract interval;
  int _cvType = -1;
  enum COLORID _colorid = COLORID_UNKNOWN;
  int _bpp = 0;

  enum cap_method
  {
    cap_method_read = 0,
    cap_method_mmap = V4L2_MEMORY_MMAP,
    cap_method_userptr = V4L2_MEMORY_USERPTR,
    cap_method_overlay = V4L2_MEMORY_OVERLAY,
    cap_method_dmabuf = V4L2_MEMORY_DMABUF,
  } cap_method_ = cap_method_mmap;

};

} /* namespace serimager */

#endif /* __QV4L2Camera_h__ */
