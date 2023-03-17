/*
 * QFFMPEGCamera.cc
 *
 *  Created on: Mar 16, 2023
 *      Author: amyznikov
 */

#include "QFFMPEGCamera.h"

namespace serimager {

QFFMPEGCamera::QFFMPEGCamera(const QString & name, QObject * parent) :
    ThisClass(name, "", "", parent)
{
}

QFFMPEGCamera::QFFMPEGCamera(const QString & name, const QString & url, const QString & opts, QObject * parent) :
    Base(parent),
    name_(name),
    url_(url),
    opts_(opts)
{
}

QFFMPEGCamera::~QFFMPEGCamera()
{
}

void QFFMPEGCamera::setName(const QString & name)
{
  name_ = name;
  Q_EMIT parametersChanged();
}

const QString& QFFMPEGCamera::name() const
{
  return name_;
}

void QFFMPEGCamera::setUrl(const QString & v)
{
  url_ = v;
  Q_EMIT parametersChanged();
}

const QString & QFFMPEGCamera::url() const
{
  return url_;
}

void QFFMPEGCamera::setOpts(const QString & v)
{
  opts_ = v;
  Q_EMIT parametersChanged();
}

const QString & QFFMPEGCamera::opts() const
{
  return opts_;
}

QFFMPEGCamera::sptr QFFMPEGCamera::create(const QString & name, const QString & url, const QString & opts, QObject * parent)
{
  ThisClass::sptr camera(new ThisClass(name, url, opts, parent));
  return camera;
}

QString QFFMPEGCamera::display_name() const
{
  return name_;
}

QString QFFMPEGCamera::parameters() const
{
  return opts_;
}

bool QFFMPEGCamera::is_same_camera(const QImagingCamera::sptr & rhs) const
{
  const ThisClass *rhsp =
      dynamic_cast<const ThisClass*>(rhs.get());

  return rhsp == this;
}

int QFFMPEGCamera::drops() const
{
  return 0;
}

bool QFFMPEGCamera::device_is_connected() const
{
  return ffmpeg_.is_open();
}

bool QFFMPEGCamera::device_connect()
{
  if( !ffmpeg_.is_open() ) {

    bool fOk =
        ffmpeg_.open(url_.toStdString(),
            opts_.toStdString());

    if( !fOk ) {
      CF_ERROR("ffmpeg_.open() fails");
      return false;
    }
  }

  return true;
}

void QFFMPEGCamera::device_disconnect()
{
  return ffmpeg_.close();
}

bool QFFMPEGCamera::device_start()
{
  if ( !ffmpeg_.is_open() ) {
    CF_ERROR("ERROR: ffmpeg_ is not open");
    return false;
  }

  cv::Mat frame;
  int64_t pts;

  if( !ffmpeg_.read(frame, &pts) ) {
    CF_ERROR("ffmpeg_.read() fails");
    return false;
  }

  CF_DEBUG("frame: %dx%d channels=%d",
      frame.cols,
      frame.rows,
      frame.channels());

  enum COLORID colorid =
      frame.channels() == 1 ?
          COLORID_MONO :
          COLORID_BGR;

  if ( !create_frame_buffers(frame.size(), frame.type(), colorid, 8, 4) ) {
    CF_ERROR("create_frame_buffers() fails");
    return false;
  }

  return true;
}

void QFFMPEGCamera::device_stop()
{
  return ;
}

int QFFMPEGCamera::device_max_qsize()
{
  return p_.size() / 2;
}

void QFFMPEGCamera::device_release_frame(const QCameraFrame::sptr & frame)
{
  qpool(frame);
}

QCameraFrame::sptr QFFMPEGCamera::device_recv_frame()
{
  QCameraFrame::sptr frm =
      dqpool();

  if( frm ) {

    int64_t pts;

    if( !ffmpeg_.read(frm->image(), &pts) ) {
      CF_ERROR("ffmpeg_.read() fails");
      qpool(frm);
      return nullptr;
    }

    frm->set_ts(pts);
  }

  return frm;
}

bool QFFMPEGCamera::create_frame_buffers(const cv::Size & imageSize,
    int cvType,
    enum COLORID colorid,
    int bpp,
    int num_buffers)
{
  p_.clear();

  for( int i = 0; i < num_buffers; ++i ) {
    p_.emplace_back(QCameraFrame::create(imageSize,
        cvType, colorid, bpp));
  }

  return true;
}

void QFFMPEGCamera::qpool(const QCameraFrame::sptr & frame)
{
  if( frame ) {
    p_.emplace_back(frame);
  }
}

QCameraFrame::sptr QFFMPEGCamera::dqpool()
{
  if( p_.empty() ) {
    CF_ERROR("APP BUG: frame pool pool devastation, must not happen");
    return nullptr;
  }

  QCameraFrame::sptr frm =
      p_.back();

  p_.pop_back();

  return frm;
}


} /* namespace serimager */
