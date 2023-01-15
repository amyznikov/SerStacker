/*
 * QSerVideoWriter.cc
 *
 *  Created on: Dec 29, 2022
 *      Author: amyznikov
 */

#include "QCameraWriter.h"
#include <core/io/save_image.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/io/c_ser_file.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <chrono>
#include <core/debug.h>


template<>
const c_enum_member* members_of<serimager::c_capture_limits::TYPE>()
{
  using namespace serimager;

  static constexpr c_enum_member members[] = {
      { c_capture_limits::ByTime, "ByTime", "" },
      { c_capture_limits::ByNumberOfFrames, "ByNumberOfFrames", "" },
      { c_capture_limits::ByTime }
  };

  return members;
}


namespace serimager {

namespace {

struct c_video_frame_writer
{

  virtual bool create(const QString & filename, int image_width, int image_height,
      enum COLORID color_id, int bits_per_plane) = 0;
  virtual bool is_open() const = 0;
  virtual bool write(const QCameraFrame::sptr & frame) = 0;
  virtual bool close() = 0;
  virtual ~c_video_frame_writer() = default;
};

struct c_ser_file_writer: c_video_frame_writer
{
  c_ser_writer ser;

  bool create(const QString & filename, int image_width, int image_height,
      enum COLORID color_id, int bits_per_plane) override
  {
    return ser.create(filename.toStdString(),
        image_width,
        image_height,
        color_id,
        bits_per_plane);
  }

  bool is_open() const override
  {
    return ser.is_open();
  }

  bool write(const QCameraFrame::sptr & frame) override
  {
    return ser.write(frame->image(), frame->ts());
  }

  virtual bool close() override
  {
    return ser.close();
  }

  ~c_ser_file_writer()
  {
    close();
  }
};

static QString getCurrentDateTimeString()
{
  struct timespec t;
  struct tm *tm;

  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
  //int msec;

  clock_gettime(CLOCK_REALTIME, &t);
  tm = gmtime(&t.tv_sec);

  year = tm->tm_year + 1900;
  month = tm->tm_mon + 1;
  day = tm->tm_mday;
  hour = tm->tm_hour;
  min = tm->tm_min;
  sec = tm->tm_sec;
  // msec = t.tv_nsec / 1000000;

  char buf[256] = "";

  snprintf(buf, sizeof(buf) - 1, "%0.4d%0.2d%0.2d_%0.2d%0.2d%0.2d_GMT",
      year, month, day, hour, min, sec);

  return buf;
}

} // namespace

QString toQString(const c_capture_limits & limits)
{
  if( limits.value < 0 ) {
    return "Unlimited";
  }

  switch (limits.type) {
    case c_capture_limits::ByTime:
      return QString("%1 sec").arg(limits.value);
    case c_capture_limits::ByNumberOfFrames:
      return QString("%1 frames").arg(limits.value);
  }

  return QString("APP BUG: invalid limits.type=%1").arg((int) limits.type);
}

QCameraWriter::QCameraWriter(QObject * parent) :
    Base(parent)
{
  //static const int a1 = qRegisterMetaType<serimager::QCameraWriter::State>("State");
  static const int a2 = qRegisterMetaType<serimager::QCameraWriter::State>("QCameraWriter::State");
  static const int a3 = qRegisterMetaType<serimager::QCameraWriter::State>("serimager::QCameraWriter::State");

  //static const int b1 = qRegisterMetaType<serimager::QCameraWriter::FORMAT>("FORMAT");
  static const int b2 = qRegisterMetaType<serimager::QCameraWriter::FORMAT>("QCameraWriter::FORMAT");
  static const int b3 = qRegisterMetaType<serimager::QCameraWriter::FORMAT>("serimager::QCameraWriter::FORMAT");
}

void QCameraWriter::setCamera(const QImagingCamera::sptr & camera)
{
  c_unique_lock lock(mtx_);

  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }

  if( (camera_ = camera) ) {
    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged);
  }

  Q_EMIT stateChanged();
}

const QImagingCamera::sptr& QCameraWriter::camera() const
{
  return camera_;
}

void QCameraWriter::setCaptureLimits(const c_capture_limits & limits)
{
  capture_limits_ = limits;
}

const c_capture_limits& QCameraWriter::captureLimits() const
{
  return capture_limits_;
}

void QCameraWriter::setOutputDirectoty(const QString & path)
{
  output_directoty_ = path;
}

const QString& QCameraWriter::outputDirectoty() const
{
  return output_directoty_;
}

void QCameraWriter::setOutputFormat(FORMAT v)
{
  output_format_ = v;
}

QCameraWriter::FORMAT QCameraWriter::outputFormat() const
{
  return output_format_;
}

void QCameraWriter::setNumRounds(int v)
{
  numRounds_ = v;
}

int QCameraWriter::numRounds() const
{
  return numRounds_;
}

void QCameraWriter::setIntervalBetweenRounds(int v)
{
  interval_between_rounds_ = v;
}

int QCameraWriter::intervalBetweenRounds() const
{
  return interval_between_rounds_;
}

int QCameraWriter::num_saved_frames() const
{
  return num_saved_frames_;
}

int QCameraWriter::num_dropped_frames() const
{
  return num_dropped_frames_;
}

double QCameraWriter::capture_duration() const
{
  return capture_duration_;
}

int QCameraWriter::round() const
{
  return round_;
}

QCameraWriter::State QCameraWriter::state() const
{
  return current_state_;
}

void QCameraWriter::setState(State state)
{
  if( current_state_ != state ) {

    current_state_ = state;

    Q_EMIT stateChanged();
  }
}

void QCameraWriter::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  Q_EMIT stateChanged();
}

void QCameraWriter::start()
{
  c_unique_lock lock(mtx_);

  if( current_state_ == State::Idle ) {

    setState(State::Starting);

    std::thread(&ThisClass::writerThreadProc,
        this).detach();
  }
}

void QCameraWriter::stop()
{
  c_unique_lock lock(mtx_);

  if( current_state_ == State::Active ) {

    setState(State::Stopping);
  }

}

void QCameraWriter::writerThreadProc()
{
  c_unique_lock lock(mtx_);

  QImagingCamera::sptr camera =
      this->camera_;

  if( !camera ) {
    CF_ERROR("No camera");
    setState(State::Idle);
    return;
  }

  if( camera->state() == QImagingCamera::State_disconnected ) {

    if( !camera->connect() ) {
      CF_ERROR("camera->connect() fails");
      setState(State::Idle);
      return;
    }

    while (camera->state() == QImagingCamera::State_connecting) {
      usleep(20 * 1000);
    }

    if( camera->state() < QImagingCamera::State_connected ) {
      CF_ERROR("Can not connect camera");
      setState(State::Idle);
      return;
    }
  }

  if( camera->state() == QImagingCamera::State_connected ) {
    if( !camera->start() ) {
      CF_ERROR("camera->start() fails");
      setState(State::Idle);
      return;
    }

    while (camera->state() == QImagingCamera::State_starting) {
      usleep(10 * 1000);
    }
  }

  if( camera->state() != QImagingCamera::State_started ) {
    CF_ERROR("Can not start camera");
    setState(State::Idle);
    return;
  }

  setState(State::Active);

  QImagingCamera *c =
      camera.get();

  std::condition_variable_any &condvar =
      c->condvar();


  const auto wakeup_condition =
      [this, c]() -> bool {

        if ( current_state_ != State::Active ) {
          return true;
        }
        if ( c->state() != QImagingCamera::State_started ) {
          return true;
        }
        if ( !c->deque().empty() && c->deque().back()->index() > last_index_ ) {
          return true;
        }
        return false;
      };

  for( round_ = 0; round_ < numRounds_ ; ++round_ ) {

    c_video_frame_writer *writer =
        nullptr;

    last_index_ = -1;
    num_saved_frames_ = 0;
    num_dropped_frames_ = 0;

    auto start_time =
        std::chrono::system_clock::now();

    double start_ts = 0;
    double last_ts = 0;
    int start_index = -1;

    const double max_capture_duration =
        capture_limits_.type == c_capture_limits::ByTime ?
            capture_limits_.value * 1000 :
            -1;

    Q_EMIT statusUpdate();

    while (current_state_ == State::Active) {

      lock.unlock();

      QImagingCamera::shared_lock slock(
          camera->mutex());

      const auto now =
          std::chrono::system_clock::now();

      if( std::chrono::duration_cast<std::chrono::seconds>(now - start_time) >= std::chrono::seconds(1) ) {
        start_time = now;
        Q_EMIT statusUpdate();
      }

      const auto timeout =
          std::chrono::milliseconds(100);

      condvar.wait_until(slock, now + timeout,
          wakeup_condition);

      lock.lock();

      if( current_state_ != State::Active ) {
        break;
      }

      if( c->state() != QImagingCamera::State_started ) {
        break;
      }

      const std::deque<QCameraFrame::sptr> &deque =
          c->deque();

      bool fok = true;

      for( const QCameraFrame::sptr &frame : deque ) {
        if( frame->index() > last_index_ ) {

          if( !writer ) {

            if( output_directoty_.isEmpty() ) {
              output_directoty_ =
                  "./capture";
            }

            if( !create_path(output_directoty_.toStdString()) ) {
              CF_ERROR("create_path('%s') fails: %s",
                  output_directoty_.toStdString().c_str(),
                  strerror(errno));
              fok = false;
              break;
            }

            output_file_name_ =
                QString("%1/%2.ser")
                    .arg(output_directoty_)
                    .arg(getCurrentDateTimeString());

            writer = new c_ser_file_writer();

            fok = writer->create(output_file_name_,
                frame->image().cols,
                frame->image().rows,
                frame->colorid(),
                frame->bpp());

            if( !fok ) {
              CF_ERROR("writer->create('%s') fails",
                  output_file_name_.toStdString().c_str());
              break;
            }

          }

          last_index_ = frame->index();
          if( start_index < 0 ) {
            start_index = last_index_;
          }

          last_ts = frame->ts();
          if( !num_saved_frames_ ) {
            start_ts = last_ts;
          }

          if( !(fok = writer->write(frame)) ) {
            CF_ERROR("writer->write('%s') fails",
                output_file_name_.toStdString().c_str());
            break;
          }

          num_dropped_frames_ =
              last_index_ - start_index - num_saved_frames_;

          ++num_saved_frames_;

          capture_duration_ =
              last_ts - start_ts;

        }
      }

      if( !fok ) {
        break;
      }

      if( capture_limits_.value > 0 ) {

        if( capture_limits_.type == c_capture_limits::ByTime ) {
          if( capture_duration_ >= max_capture_duration ) {
            break;
          }
        }
        else if( capture_limits_.type == c_capture_limits::ByNumberOfFrames ) {
          if( num_saved_frames_ >= capture_limits_.value ) {
            break;
          }
        }
      }
    }

    if( writer ) {
      writer->close();
      delete writer;
    }

    Q_EMIT statusUpdate();

    for ( int i = 0; i < interval_between_rounds_; ++i ) {
      if ( current_state_ != State::Active || c->state() != QImagingCamera::State_started ) {
        break;
      }
      sleep(1);
    }

    if ( current_state_ != State::Active || c->state() != QImagingCamera::State_started ) {
      break;
    }

  }

  setState(State::Idle);
  Q_EMIT statusUpdate();
}

} /* namespace serimager
 */
