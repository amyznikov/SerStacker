/*
 * QFFMPEGCamera.h
 *
 *  Created on: Mar 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFFMPEGCamera_h__
#define __QFFMPEGCamera_h__

#include "QImagingCamera.h"
#include <core/io/c_ffmpeg_file.h>

namespace serimager {

class QFFMPEGCamera :
    public QImagingCamera
{
  Q_OBJECT;

public:
  typedef QFFMPEGCamera ThisClass;
  typedef QImagingCamera Base;
  typedef std::shared_ptr<ThisClass> sptr;

  ~QFFMPEGCamera();

  static sptr create(const QString & name,
      const QString & url,
      const QString & opts,
      QObject * parent =  nullptr);

  void setName(const QString & name);
  const QString & name() const;

  void setUrl(const QString & v);
  const QString & url() const;

  void setOpts(const QString & v);
  const QString & opts() const;

  QString display_name() const override;
  QString parameters() const override;

  bool is_same_camera(const QImagingCamera::sptr & rhs) const override;
  int drops() const override;

Q_SIGNALS:
  void parametersChanged();

protected:
  QFFMPEGCamera(const QString & name, const QString & url, const QString & opts = "", QObject * parent = nullptr);
  QFFMPEGCamera(const QString & name, QObject * parent = nullptr);

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

  void qpool(const QCameraFrame::sptr & );
  QCameraFrame::sptr dqpool();

protected:
  c_ffmpeg_reader _ffmpeg;

  QString _name;
  QString _url;
  QString _opts = "-nobuffer -nodelay";
  std::vector<QCameraFrame::sptr> _p;
};

/**
 * This struct is just used to serialize QFFMPEGCamera parameters
 * into QSettings data stream
 * */
struct QFFMPEGCameraParameters {
  QString name;
  QString url;
  QString opts;
};


} /* namespace serimager */



/*
 * must be declared outside of any namespace
 * */

Q_DECLARE_METATYPE(serimager::QFFMPEGCamera::sptr);
Q_DECLARE_METATYPE(serimager::QFFMPEGCameraParameters);
Q_DECLARE_METATYPE(QList<serimager::QFFMPEGCameraParameters>);

inline QDataStream& operator << (QDataStream & out, const serimager::QFFMPEGCameraParameters & v)
{
  return (out << v.name << v.url << v.opts);
}

inline QDataStream& operator >> (QDataStream & in, serimager::QFFMPEGCameraParameters & v)
{
  return (in >> v.name >> v.url >> v.opts);
}

inline QDataStream& operator << (QDataStream & out, const QList<serimager::QFFMPEGCameraParameters> & list)
{
  out << list.size();
  for( int i = 0, n = list.size(); i < n; ++i ) {
    out << list[i];
  }
  return out;
}

inline QDataStream& operator >> (QDataStream & in, QList<serimager::QFFMPEGCameraParameters> & list)
{
  list.clear();

  int n = 0;
  if( (in >> n).status() == QDataStream::Ok ) {
    for( int i = 0; i < n; ++i ) {
      serimager::QFFMPEGCameraParameters p;
      if( (in >> p).status() != QDataStream::Ok ) {
        break;
      }
      list.append(p);
    }
  }
  return in;
}

#endif /* __QFFMPEGCamera_h__ */
