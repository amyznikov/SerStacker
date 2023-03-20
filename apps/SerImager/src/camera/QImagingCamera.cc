/*
 * QImagingCamera.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QImagingCamera.h"
#include <core/ssprintf.h>
#include <core/get_time.h>

template<>
const c_enum_member* members_of<serimager::QImagingCamera::State>()
{
  using namespace serimager;

  static constexpr c_enum_member members[] = {
      { QImagingCamera::State_disconnected, "disconnected", "" },
      { QImagingCamera::State_connecting, "connecting", "" },
      { QImagingCamera::State_connected, "connected", "" },
      { QImagingCamera::State_starting, "starting", "" },
      { QImagingCamera::State_started, "started", "" },
      { QImagingCamera::State_stopping, "stopping", "" },
      { QImagingCamera::State_disconnecting, "disconnecting", "" },
      { QImagingCamera::State_disconnected }
  };

  return members;
}

template<>
const c_enum_member* members_of<serimager::QImagingCamera::ExposureStatus>()
{
  using namespace serimager;

  static constexpr c_enum_member members[] = {
      { QImagingCamera::Exposure_idle, "idle", "idle state, can start exposure now" },
      { QImagingCamera::Exposure_working, "exposing", "exposing" },
      { QImagingCamera::Exposure_success, "success", "exposure finished and waiting for download" },
      { QImagingCamera::Exposure_failed, "failed", "exposure failed, you need to start exposure again" },
      { QImagingCamera::Exposure_idle }
  };

  return members;
}


namespace serimager {



std::string fourccToString(uint32_t fourcc)
{
  std::string s;

  const char *p =
      (const char*) &fourcc;

  for( int i = 0; i < 4 && p[i]; ++i ) {
    s += p[i];
  }
  return s;
}

QString fourccToQString (uint32_t fourcc)
{
  QString s;

  const char *p =
      (const char*) &fourcc;

  for( int i = 0; i < 4 && p[i]; ++i ) {
    s += p[i];
  }
  return s;

}


QImagingCamera::QImagingCamera(QObject * parent) :
    Base(parent)
{
  //static const int r1 = qRegisterMetaType<serimager::QImagingCamera::State>("State");
  static const int r2 = qRegisterMetaType<serimager::QImagingCamera::State>("QImagingCamera::State");
  static const int r3 = qRegisterMetaType<serimager::QImagingCamera::State>("serimager::QImagingCamera::State");
  static const int r4 = qRegisterMetaType<serimager::QImagingCamera::State>("QImagingCamera::ExposureStatus");
  static const int r5 = qRegisterMetaType<serimager::QImagingCamera::State>("serimager::QImagingCamera::ExposureStatus");
}


QImagingCamera::~QImagingCamera()
{
  finish();
}

void QImagingCamera::finish()
{
  stop();

  while ( current_state_ > State_connected ) {
    usleep(100*1000);
  }

  disconnect();

  while ( current_state_ > State_disconnected ) {
    usleep(100*1000);
  }
}


QString QImagingCamera::parameters() const
{
  return QString();
}

QImagingCamera::State QImagingCamera::state() const
{
  return current_state_;
}

const QString & QImagingCamera::reason() const
{
  return stateChangeReason_;
}

void QImagingCamera::setState(State newState, const QString & reason)
{
  if( current_state_ != newState ) {

    const State oldState =
        current_state_;

    current_state_ = newState;

    if( !reason.isEmpty() ) {
      stateChangeReason_ = reason;
    }

    onStateCanged(oldState, newState);
  }
}

void QImagingCamera::onStateCanged(State oldSate, State newState)
{
  Q_EMIT stateChanged(oldSate, newState);
}


std::shared_mutex & QImagingCamera::mutex()
{
  return mtx_;
}

std::condition_variable_any & QImagingCamera::condvar()
{
  return condvar_;
}

const std::deque<QCameraFrame::sptr> & QImagingCamera::deque() const
{
  return deque_;
}

void QImagingCamera::setRoi(const QRect & roi)
{
  roi_ = roi;
  Q_EMIT roiChanged(roi);
}

const QRect & QImagingCamera::roi() const
{
  return roi_;
}

bool QImagingCamera::connect()
{
  unique_lock lock(mtx_);

  if( current_state_ != State_disconnected ) {
    CF_ERROR("QImagingCamera: inappropriate state: %s",
        toString(current_state_));
    return false;
  }

  setState(State_connecting);

  std::thread([this]() {

    errno = 0;
    device_connect();

    unique_lock lock(mtx_);

    if ( current_state_ == State_connecting && device_is_connected() ) {
      setState(State_connected);
    }
    else {
      setState(State_disconnected,
          strerror(errno));
    }

  }).detach();

  return true;
}

void QImagingCamera::disconnect()
{
  unique_lock lock(mtx_);

  switch (current_state_) {
    case State_disconnected:
      case State_disconnecting:
      break;

      case State_connected:
      case State_starting:
      case State_started:
        setState(State_disconnecting);
        lock.unlock();
        device_disconnect();
        lock.lock();
        setState(State_disconnected);
        break;

    case State_connecting:
      case State_stopping:
      break;
  }
}

bool QImagingCamera::start()
{
  unique_lock lock(mtx_);

  switch (current_state_) {
    case State_disconnected:
    case State_disconnecting:
    case State_connecting:
    case State_stopping:
    CF_ERROR("Inappropriate state: %s",
        toString(current_state_));
    return false;

    case State_starting:
    case State_started:
    return true;

    case State_connected:
      break;
  }

  setState(State_starting);

  std::thread([this]() {

    int index = 0;

    if( !device_start() ) {
      unique_lock lock(mtx_);
      setState(device_is_connected() ? State_connected : State_disconnected,
          "device_start() fails");
      return;
    }

    unique_lock lock(mtx_);
    if ( current_state_ != State_starting ) {
      setState(device_is_connected() ? State_connected : State_disconnected);
      return;
    }

    setState(State_started);

    const int qsize =
        device_max_qsize();

    while ( current_state_ == State_started ) {

      INSTRUMENT_REGION("mainloop");

      lock.unlock();

      QCameraFrame::sptr frame =
          device_recv_frame();

      lock.lock();

      if ( !frame ) {
        CF_ERROR("device_recv_frame() fails");
        break;
      }

      frame->set_index(index++);
      frame->set_ts(get_realtime_ms());

      deque_.emplace_back(frame);

      if ( deque_.size() > qsize ) {

        frame = deque_.front();
        deque_.pop_front();

        device_release_frame(frame);
      }

      condvar_.notify_all();
    }

    device_stop();
    deque_.clear();
    condvar_.notify_all();

    setState(device_is_connected() ? State_connected : State_disconnected,
        strerror(errno));


  }).detach();

  return true;
}

void QImagingCamera::stop()
{
  unique_lock lock(mtx_);

  switch (current_state_) {
    case State_disconnected:
      case State_disconnecting:
      case State_connected:
      case State_stopping:
      break;

    case State_connecting:
      case State_starting:
      case State_started:
      setState(State_stopping);
      break;
  }
}


} /* namespace qserimager */
