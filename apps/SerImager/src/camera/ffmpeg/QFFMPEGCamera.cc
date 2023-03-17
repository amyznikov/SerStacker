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
  return false;
}

void QFFMPEGCamera::device_disconnect()
{
  return ;
}

bool QFFMPEGCamera::device_start()
{
  return false;
}

void QFFMPEGCamera::device_stop()
{
  return ;
}

int QFFMPEGCamera::device_max_qsize()
{
  return 0;
}

void QFFMPEGCamera::device_release_frame(const QCameraFrame::sptr & queue)
{
  return ;
}

QCameraFrame::sptr QFFMPEGCamera::device_recv_frame()
{
  return nullptr;
}


} /* namespace serimager */
