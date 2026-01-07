/*
 * QImagingCamera.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QImagingCamera.h"
#include <core/ssprintf.h>
#include <core/get_time.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<serimager::QImagingCamera::State>()
{
  using namespace serimager;

  static const c_enum_member members[] = {
      { QImagingCamera::State_disconnected, "disconnected", "" },
      { QImagingCamera::State_connecting, "connecting", "" },
      { QImagingCamera::State_connected, "connected", "" },
      { QImagingCamera::State_starting, "starting", "" },
      { QImagingCamera::State_started, "started", "" },
      { QImagingCamera::State_stop, "stopping", "" },
      { QImagingCamera::State_disconnect, "disconnecting", "" },
      { QImagingCamera::State_disconnected }
  };

  return members;
}

template<>
const c_enum_member* members_of<serimager::QImagingCamera::ExposureStatus>()
{
  using namespace serimager;

  static const c_enum_member members[] = {
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

  while ( _current_state > State_connected ) {
    usleep(100*1000);
  }

  disconnect();

  while ( _current_state > State_disconnected ) {
    usleep(100*1000);
  }
}


QString QImagingCamera::parameters() const
{
  return QString();
}

QImagingCamera::State QImagingCamera::state() const
{
  return _current_state;
}

const QString & QImagingCamera::reason() const
{
  return _stateChangeReason;
}

void QImagingCamera::setState(State newState, const QString & reason)
{
  if( _current_state != newState ) {

    const State oldState =
        _current_state;

    _current_state = newState;

    if( !reason.isEmpty() ) {
      _stateChangeReason = reason;
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
  return _mtx;
}

std::condition_variable_any & QImagingCamera::condvar()
{
  return _condvar;
}

const std::deque<QCameraFrame::sptr> & QImagingCamera::deque() const
{
  return _deque;
}

void QImagingCamera::setRoi(const QRect & roi)
{
  _roi = roi;
  Q_EMIT roiChanged(roi);
}

const QRect & QImagingCamera::roi() const
{
  return _roi;
}

bool QImagingCamera::connect()
{
  unique_lock lock(_mtx);

  if( _current_state != State_disconnected ) {
    CF_ERROR("QImagingCamera: inappropriate state: %s",
        toCString(_current_state));
    return false;
  }

  setState(State_connecting);

  std::thread([this]() {

    errno = 0;
    device_connect();

    unique_lock lock(_mtx);

    if ( !device_is_connected() ) {
      setState(State_disconnected, strerror(errno));
    }
    else if (_current_state != State_connecting && _current_state != State_connected ) { // interrupted externally
      lock.unlock();
      device_disconnect();
      lock.lock();
      setState(State_disconnected, "interrupted");
    }
    else if (_current_state != State_connected ){
      setState(State_connected);
    }

  }).detach();

  return true;
}



bool QImagingCamera::start()
{
  unique_lock lock(_mtx);

  if( _current_state == State_starting || _current_state == State_started ) {
    return true;
  }

  if( _current_state != State_connected ) {
    CF_ERROR("Inappropriate state: %s",
        toCString(_current_state));
    return false;
  }

  setState(State_starting);

  bool start_fails = false;
  for ( const auto & onstarting : _prestartproc ) {
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
      switch (_current_state) {
        case State_disconnected:
        if ( device_is_connected() ) {
          disconnect();
        }
        break;

        case State_connecting:
        CF_ERROR("FATAL APP BUG: Unexpected state '%s'", toCString(_current_state));
        device_disconnect();
        setState(State_disconnected);
        break;

        case State_connected:
        if ( !device_is_connected() ) {
          CF_ERROR("FATAL APP BUG: Device is not connected but current state state is '%s'",
              toCString(_current_state));
        }
        break;

        case State_starting:
        setState(device_is_connected() ? State_connected : State_disconnected);
        break;

        case State_started:
          setState(device_is_connected() ? State_connected : State_disconnected);
        break;

        case State_stop:
        setState(device_is_connected() ? State_connected : State_disconnected);
        break;

        case State_disconnect:
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
      unique_lock lock(_mtx);
      finish_thread();
      return;
    }

    unique_lock lock(_mtx);
    if ( _current_state != State_starting ) {
      CF_ERROR("interrupted start");
      finish_thread();
      return;
    }

    setState(State_started);

    const int qsize =
        device_max_qsize();

    while ( _current_state == State_started ) {

      QCameraFrame::sptr frame;
      bool fOk;

      lock.unlock();
      fOk = device_recv_frame(frame);
      lock.lock();
      if ( !fOk ) {
        CF_ERROR("device_recv_frame() fails");
        break;
      }

      if ( frame ) {

        frame->set_index(index++);
        frame->set_ts(get_realtime_sec());

        _deque.emplace_back(frame);

        if ( _deque.size() > qsize ) {

          frame = _deque.front();
          _deque.pop_front();

          device_release_frame(frame);
        }

        _condvar.notify_all();
      }
    }

    device_stop();
    finish_thread();
    _deque.clear();
    _condvar.notify_all();

  }).detach();

  return true;
}

void QImagingCamera::stop()
{
  unique_lock lock(_mtx);

  switch (_current_state) {
    case State_disconnected:
      case State_disconnect:
      case State_connected:
      case State_stop:
      break;

    case State_connecting:
      case State_starting:
      case State_started:
      setState(State_stop);
      _condvar.notify_all();
      break;
  }
}


void QImagingCamera::disconnect()
{
  unique_lock lock(_mtx);

  //  CF_DEBUG("enter: state=%s", toString(current_state_));

  switch (_current_state) {
    case State_connecting:
      // just send signal to QImagingCamera::connect()
      setState(State_disconnect);
      break;

    case State_starting:
      // just send signal to QImagingCamera::start()
      setState(State_disconnect);
      break;

    case State_connected:
      setState(State_disconnect);
      lock.unlock();
      device_disconnect();
      lock.lock();
      setState(State_disconnected);
      break;

    case State_started:
      setState(State_disconnect);
      break;

    case State_stop:

      while (_current_state != State_connected && _current_state != State_disconnected) {
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

    case State_disconnect:
      break;

    case State_disconnected:
      break;

  }

  // CF_DEBUG("leave: state=%s device_is_connected()=%d", toString(current_state_), device_is_connected());
}

void QImagingCamera::addprestartproc(const PreStartProc * proc)
{
  for ( auto & p : _prestartproc ) {
    if ( p == proc ) {
      return;
    }
  }

  _prestartproc.emplace_back(proc);
}

void QImagingCamera::removeprestartproc(const PreStartProc * proc)
{
  for ( auto ii = _prestartproc.begin(); ii != _prestartproc.end(); ++ii ) {
    if ( *ii == proc ) {
      _prestartproc.erase(ii);
      return;
    }
  }
}



} /* namespace serimager */
