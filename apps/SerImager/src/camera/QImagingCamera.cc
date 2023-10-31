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
  // don't call finish() because of pure virtual functions calls
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

    if ( !device_is_connected() ) {
      setState(State_disconnected, strerror(errno));
    }
    else if (current_state_ != State_connecting ) { // interrupted externally
      device_disconnect();
      setState(State_disconnected, "interrupted");
    }
    else {
      setState(State_connected);
    }

  }).detach();

  return true;
}



bool QImagingCamera::start()
{
  unique_lock lock(mtx_);

  if( current_state_ == State_starting || current_state_ == State_started ) {
    return true;
  }

  if( current_state_ != State_connected ) {
    CF_ERROR("Inappropriate state: %s",
        toString(current_state_));
    return false;
  }

  setState(State_starting);

  bool start_fails = false;
  for ( const auto & onstarting : prestartproc_ ) {
    if ( onstarting && !(*onstarting)() ) {
      start_fails = true;
      break;
    }
  }

  if( start_fails ) {
    CF_ERROR("Start interrupted by prestartproc");
    setState(device_is_connected() ? State_connected : State_disconnected);
    return false;
  }

  std::thread([this]() {

    const auto finish_thread = [this]() {
      switch (current_state_) {
        case State_disconnected:
        if ( device_is_connected() ) {
          disconnect();
        }
        break;

        case State_connecting:
        CF_ERROR("FATAL APP BUG: Unexpected state '%s'", toString(current_state_));
        device_disconnect();
        setState(State_disconnected);
        break;

        case State_connected:
        if ( !device_is_connected() ) {
          CF_ERROR("FATAL APP BUG: Device is not connected but current state state is '%s'",
              toString(current_state_));
        }
        break;

        case State_starting:
        setState(device_is_connected() ? State_connected : State_disconnected);
        break;

        case State_started:
          setState(device_is_connected() ? State_connected : State_disconnected);
        break;

        case State_stopping:
        setState(device_is_connected() ? State_connected : State_disconnected);
        break;

        case State_disconnecting:
        if ( device_is_connected() ) {
          device_disconnect();
        }
        setState(State_disconnected);
        break;
      }
    };


    int index = 0;

    if( !device_start() ) {
      CF_ERROR("device_start() fails");
      unique_lock lock(mtx_);
      finish_thread();
      return;
    }

    unique_lock lock(mtx_);
    if ( current_state_ != State_starting ) {
      CF_ERROR("interrupted start");
      finish_thread();
      return;
    }

    setState(State_started);

    const int qsize =
        device_max_qsize();

    while ( current_state_ == State_started ) {

      lock.unlock();

      QCameraFrame::sptr frame =
          device_recv_frame();

      lock.lock();

      if ( !frame ) {
        CF_ERROR("device_recv_frame() fails");
        break;
      }

      frame->set_index(index++);
      frame->set_ts(get_realtime_sec());

      deque_.emplace_back(frame);

      if ( deque_.size() > qsize ) {

        frame = deque_.front();
        deque_.pop_front();

        device_release_frame(frame);
      }

      condvar_.notify_all();
    }

    device_stop();
    finish_thread();
    deque_.clear();
    condvar_.notify_all();

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


void QImagingCamera::disconnect()
{
  unique_lock lock(mtx_);

  //  CF_DEBUG("enter: state=%s", toString(current_state_));

  switch (current_state_) {
    case State_connecting:
      // just send signal to QImagingCamera::connect()
      setState(State_disconnecting);
      break;

    case State_starting:
      // just send signal to QImagingCamera::start()
      setState(State_disconnecting);
      break;

    case State_connected:
      setState(State_disconnecting);
      lock.unlock();
      device_disconnect();
      lock.lock();
      setState(State_disconnected);
      break;

    case State_started:
      setState(State_disconnecting);
      break;

    case State_stopping:

      while (current_state_ != State_connected && current_state_ != State_disconnected) {
        lock.unlock();
        // CF_DEBUG("usleep(50 * 1000): current_state_=%s", toString(current_state_));
        usleep(50 * 1000);
        lock.lock();
      }

      if( device_is_connected() ) {
        device_disconnect();
      }

      setState(State_disconnected);
      break;

    case State_disconnecting:
      break;

    case State_disconnected:
      break;

  }

  // CF_DEBUG("leave: state=%s device_is_connected()=%d", toString(current_state_), device_is_connected());
}

void QImagingCamera::addprestartproc(const PreStartProc * proc)
{
  for ( auto & p : prestartproc_ ) {
    if ( p == proc ) {
      return;
    }
  }

  prestartproc_.emplace_back(proc);
}

void QImagingCamera::removeprestartproc(const PreStartProc * proc)
{
  for ( auto ii = prestartproc_.begin(); ii != prestartproc_.end(); ++ii ) {
    if ( *ii == proc ) {
      prestartproc_.erase(ii);
      return;
    }
  }
}



} /* namespace serimager */
