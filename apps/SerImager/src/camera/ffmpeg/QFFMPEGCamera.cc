/*
 * QFFMPEGCamera.cc
 *
 *  Created on: Mar 16, 2023
 *      Author: amyznikov
 */

#include "QFFMPEGCamera.h"
#include <core/debug.h>

namespace serimager {

QFFMPEGCamera::QFFMPEGCamera(const QString & name, QObject * parent) :
    ThisClass(name, "", "", parent)
{
}

QFFMPEGCamera::QFFMPEGCamera(const QString & name, const QString & url, const QString & opts, QObject * parent) :
    Base(parent),
    _name(name),
    _url(url),
    _opts(opts)
{
}

QFFMPEGCamera::~QFFMPEGCamera()
{
  finish();
  _ffmpeg.close();
}

void QFFMPEGCamera::setName(const QString & name)
{
  _name = name;
  Q_EMIT parametersChanged();
}

const QString& QFFMPEGCamera::name() const
{
  return _name;
}

void QFFMPEGCamera::setUrl(const QString & v)
{
  _url = v;
  Q_EMIT parametersChanged();
}

const QString & QFFMPEGCamera::url() const
{
  return _url;
}

void QFFMPEGCamera::setOpts(const QString & v)
{
  _opts = v;
  Q_EMIT parametersChanged();
}

const QString & QFFMPEGCamera::opts() const
{
  return _opts;
}

QFFMPEGCamera::sptr QFFMPEGCamera::create(const QString & name, const QString & url, const QString & opts, QObject * parent)
{
  ThisClass::sptr camera(new ThisClass(name, url, opts, parent));
  return camera;
}

QString QFFMPEGCamera::display_name() const
{
  return _name;
}

QString QFFMPEGCamera::parameters() const
{
  return _opts;
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
  return _ffmpeg.is_open();
}

bool QFFMPEGCamera::device_connect()
{
  if( !_ffmpeg.is_open() ) {

    bool fOk =
        _ffmpeg.open(_url.toStdString(),
            _opts.toStdString());

    if( !fOk ) {
      CF_ERROR("ffmpeg_.open() fails");
      return false;
    }
  }

  return true;
}

void QFFMPEGCamera::device_disconnect()
{
  _ffmpeg.close();

}

bool QFFMPEGCamera::device_start()
{
  if ( !_ffmpeg.is_open() ) {
    CF_ERROR("ERROR: ffmpeg_ is not open");
    return false;
  }

  cv::Mat frame;
  double pts;

  if( !_ffmpeg.read(frame, &pts) ) {
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
  return _p.size() / 2;
}

void QFFMPEGCamera::device_release_frame(const QCameraFrame::sptr & frame)
{
  qpool(frame);
}

bool QFFMPEGCamera::device_recv_frame(QCameraFrame::sptr & frm)
{
  if( (frm = dqpool()) ) {

    double pts;

    if( _ffmpeg.read(frm->image(), &pts) ) {
      frm->set_ts(pts);
      return true;
    }

    CF_ERROR("ffmpeg.read() fails");
    qpool(frm);
    frm.reset();
  }

  return false;
}

bool QFFMPEGCamera::create_frame_buffers(const cv::Size & imageSize,
    int cvType,
    enum COLORID colorid,
    int bpp,
    int num_buffers)
{
  _p.clear();

  for( int i = 0; i < num_buffers; ++i ) {
    _p.emplace_back(QCameraFrame::create(imageSize,
        cvType, colorid, bpp));
  }

  return true;
}

void QFFMPEGCamera::qpool(const QCameraFrame::sptr & frame)
{
  if( frame ) {
    _p.emplace_back(frame);
  }
}

QCameraFrame::sptr QFFMPEGCamera::dqpool()
{
  if( _p.empty() ) {
    CF_ERROR("APP BUG: frame pool pool devastation, must not happen");
    return nullptr;
  }

  QCameraFrame::sptr frm =
      _p.back();

  _p.pop_back();

  return frm;
}


} /* namespace serimager */
