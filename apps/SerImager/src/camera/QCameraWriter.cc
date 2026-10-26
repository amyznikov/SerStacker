/*
 * QSerVideoWriter.cc
 *
 *  Created on: Dec 29, 2022
 *      Author: amyznikov
 */

#include "QCameraWriter.h"
#include <gui/widgets/qsprintf.h>
#include <core/io/save_image.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/io/c_ser_file.h>
#include <core/get_time.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <chrono>
#include <core/debug.h>


template<>
const c_enum_member* members_of<serimager::c_capture_limits::TYPE>()
{
  using namespace serimager;

  static const c_enum_member members[] = {
      { c_capture_limits::ByTime, "ByTime", "" },
      { c_capture_limits::ByNumberOfFrames, "ByNumberOfFrames", "" },
      { c_capture_limits::ByTime }
  };

  return members;
}

template<>
const c_enum_member* members_of<serimager::QCameraWriter::FORMAT>()
{
  using namespace serimager;
  static const c_enum_member members[] = {
      { QCameraWriter::SER, "SER", "" },
      { QCameraWriter::AVI, "AVI", "" },
      { QCameraWriter::IMAGES, "IMAGES", "" },
      { QCameraWriter::SER }
  };

  return members;
}


namespace serimager {

namespace {

class c_video_frame_writer
{
public:
  virtual bool is_open() const = 0;
  virtual bool write(const QCameraFrame::sptr & frame) = 0;
  virtual void close() = 0;
  virtual ~c_video_frame_writer() = default;
};

struct c_ser_file_writer:
    c_video_frame_writer
{
  c_ser_writer ser;

  bool create(const QString & filename, const cv::Size & frame_size, enum COLORID color_id, int bits_per_plane)
  {
    return ser.create(filename.toStdString(),
        frame_size.width,
        frame_size.height,
        color_id,
        bits_per_plane);
  }

  bool is_open() const override
  {
    return ser.is_open();
  }

  bool write(const QCameraFrame::sptr & frame) override
  {
    return ser.write(frame->image(), cv::noArray(), realtime_sec_to_ser_timestamp(frame->ts()));
  }

  virtual void close() override
  {
    ser.close();
  }

  ~c_ser_file_writer()
  {
    close();
  }
};

struct c_avi_file_writer:
    c_video_frame_writer
{
  c_ffmpeg_writer ffmpeg;
  double tscale_ = 1;
  double start_ts = 0;

  // "-c huffyuv -r 100 -f avi"
  // "-r 10 -c rawvideo -pix_fmt rgb24";
  bool create(const QString & filename, const QString & ffopts, const cv::Size & frame_size,
      enum COLORID color_id, int /*bits_per_plane*/)
  {
    const bool is_color =
        color_id == COLORID_RGB || color_id == COLORID_BGR;

    if ( !ffmpeg.open(filename.toStdString(), frame_size, is_color, ffopts.toStdString())) {
      CF_ERROR("ffmpeg.open('%s', opts='%s') fails", ffmpeg.filename().c_str(), ffmpeg.opts().c_str());
      return false;
    }

    const AVStream * stream =
        ffmpeg.stream();

    tscale_ =
        (double) stream->time_base.den /
            stream->time_base.num;

    return true;
  }

  bool is_open() const override
  {
    return ffmpeg.is_open();
  }

  bool write(const QCameraFrame::sptr & frame) override
  {
    if( ffmpeg.frames_written() < 1 ) {
      start_ts = frame->ts();
    }
    return ffmpeg.write(frame->image(), (int64_t) (tscale_ * (frame->ts() - start_ts)));
  }

  virtual void close() override
  {
    ffmpeg.close();
  }

  ~c_avi_file_writer()
  {
    close();
  }
};

struct c_avi_stereo_writer:
    c_video_frame_writer
{
  c_ffmpeg_writer ffmpegs[2];
  cv::Rect src_panes[2];
  cv::Size output_size;
  stereo_stream_layout_type frame_layout = stereo_stream_layout_horizontal_split;
  double tscale_ = 1;
  double start_ts = 0;
  bool downscale_panes = false;

  bool create(const QString & filename, const QString & ffopts,
      const cv::Size & src_frame_size,
      stereo_stream_layout_type frame_layout,
      bool swap_cameras,
      bool downscale_panes,
      enum COLORID color_id)
  {
    const bool is_color =
        color_id == COLORID_RGB || color_id == COLORID_BGR;


    this->downscale_panes =
        downscale_panes;

    this->frame_layout =
        frame_layout;

    std::string filenames[2] = {
        ssprintf("%s.left.avi", filename.toUtf8().constData()),
        ssprintf("%s.right.avi", filename.toUtf8().constData()),
    };

    switch (frame_layout) {
      case stereo_stream_layout_horizontal_split:
        src_panes[0].x = 0;
        src_panes[0].y = 0;
        src_panes[0].width = src_frame_size.width / 2;
        src_panes[0].height = src_frame_size.height;
        src_panes[1].x = src_frame_size.width / 2;
        src_panes[1].y = 0;
        src_panes[1].width = src_frame_size.width / 2;
        src_panes[1].height = src_frame_size.height;

        if ( downscale_panes ) {
          output_size.width = src_frame_size.width / 2;
          output_size.height = src_frame_size.height / 2;
        }
        else {
          output_size.width = src_frame_size.width / 2;
          output_size.height = src_frame_size.height;
        }

        break;

      case stereo_stream_layout_vertical_split:
        src_panes[0].x = 0;
        src_panes[0].y = 0;
        src_panes[0].width = src_frame_size.width;
        src_panes[0].height = src_frame_size.height / 2;

        src_panes[1].x = 0;
        src_panes[1].y = src_frame_size.height / 2;
        src_panes[1].width = src_frame_size.width;
        src_panes[1].height = src_frame_size.height / 2;

        if ( downscale_panes ) {
          output_size.width = src_frame_size.width / 2;
          output_size.height = src_frame_size.height / 2;
        }
        else {
          output_size.width = src_frame_size.width;
          output_size.height = src_frame_size.height / 2;
        }

        break;
    }

    if( swap_cameras ) {
      std::swap(src_panes[0], src_panes[1]);
    }


    for ( int i = 0; i < 2; ++i ) {

      bool fok =
          ffmpegs[i].open(filenames[i],
              output_size,
              is_color,
              ffopts.toStdString()); // "-c huffyuv -r 10 -f avi"

      if ( !fok ) {
        CF_ERROR("ffmpegs[i=%d].open('%s') fails", i, filenames[i].c_str());
        close();
        return false;
      }
    }

    const AVStream * stream =
        ffmpegs[0].stream();

    tscale_ =
        (double) stream->time_base.den /
            stream->time_base.num;

    return true;
  }

  bool is_open() const override
  {
    return ffmpegs[0].is_open() && ffmpegs[1].is_open();
  }

  bool write(const QCameraFrame::sptr & frame) override
  {
    cv::Mat images[2];

    if( !downscale_panes ) {
      images[0] = frame->image()(src_panes[0]);
      images[1] = frame->image()(src_panes[1]);
    }
    else {
      for( int i = 0; i < 2; ++i ) {
        cv::resize(frame->image()(src_panes[i]), images[i],
            output_size,
            0, 0,
            cv::INTER_LINEAR);
      }
    }

    if( ffmpegs[0].frames_written() < 1 ) {
      start_ts = frame->ts();
    }


    for( int i = 0; i < 2; ++i ) {
      if( !ffmpegs[i].write(images[i], tscale_ * (frame->ts() - start_ts)) ) {
        CF_ERROR("ffmpegs[i=%d].write() fails", i);
        return false;
      }
    }

    return true;
  }

  virtual void close() override
  {
    for ( int i = 0; i < 2; ++i ) {
      ffmpegs[i].close();
    }
  }

  ~c_avi_stereo_writer()
  {
    close();
  }
};


struct c_text_file_writer
{
  FILE * fp = nullptr;

  c_text_file_writer()
  {
  }

  c_text_file_writer(const std::string & filename)
  {
    if( !filename.empty() ) {
      open(filename);
    }
  }

  ~c_text_file_writer()
  {
    close();
  }

  bool open(const std::string & filename)
  {
    close();

    if( !(fp = fopen(filename.c_str(), "w")) ) {
      CF_ERROR("Can not write '%s' : %s", filename.c_str(), strerror(errno));
    }
    else {
      fprintf(fp, "[SerImager]\n");
    }

    return fp != nullptr;
  }

  void close()
  {
    if ( fp ) {
      fclose(fp);
      fp = nullptr;
    }
  }

  bool is_open() const
  {
    return fp != nullptr;
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

static QString getHumanReadableCurrentDateTimeString()
{
  struct timespec t;
  struct tm *tm;

  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
  int msec;

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

  snprintf(buf, sizeof(buf) - 1, "%0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d GMT",
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
  c_unique_lock lock(_mtx);

  if( _camera ) {
    disconnect(_camera.get(), nullptr,
        this, nullptr);
  }

  if( (_camera = camera) ) {
    connect(_camera.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged);
  }

  Q_EMIT stateChanged();
}

const QImagingCamera::sptr& QCameraWriter::camera() const
{
  return _camera;
}

void QCameraWriter::set_enable_split_stereo_stream(bool v)
{
  _enableSplitStereoStream = v;
}

bool QCameraWriter::enable_split_stereo_stream() const
{
  return _enableSplitStereoStream;
}

c_stereo_stream_options & QCameraWriter::stereo_stream_options()
{
  return _stereoStreamOptions;
}

const c_stereo_stream_options & QCameraWriter::stereo_stream_options() const
{
  return _stereoStreamOptions;
}

void QCameraWriter::setCaptureLimits(const c_capture_limits & limits)
{
  _captureLimits = limits;
}

const c_capture_limits& QCameraWriter::captureLimits() const
{
  return _captureLimits;
}

void QCameraWriter::setOutputDirectory(const QString & path)
{
  _outputDirectory = path;
  Q_EMIT outputDirectoryChanged(_outputDirectory);
}

const QString& QCameraWriter::outputDirectory() const
{
  return _outputDirectory;
}

void QCameraWriter::setOutputFormat(FORMAT v)
{
  _outputFormat = v;
}

QCameraWriter::FORMAT QCameraWriter::outputFormat() const
{
  return _outputFormat;
}

void QCameraWriter::setFFmpegOptions(const QString & opts)
{
  _ffmpegOptions = opts;
}

const QString& QCameraWriter::ffmpegOptions() const
{
  return _ffmpegOptions;
}

void QCameraWriter::setFilenamePrefix(const QString & v)
{
  _filenamePrefix = v;
}

const QString & QCameraWriter::filenamePrefix() const
{
  return _filenamePrefix;
}

void QCameraWriter::setFilenameSuffix(const QString & v)
{
  _filenameSuffix = v;
}

const QString & QCameraWriter::filenameSuffix() const
{
  return _filenameSuffix;
}

void QCameraWriter::setNumRounds(int v)
{
  _numRounds = v;
}

int QCameraWriter::numRounds() const
{
  return _numRounds;
}

void QCameraWriter::setIntervalBetweenRounds(int v)
{
  _intervalBetweenRounds = v;
}

int QCameraWriter::intervalBetweenRounds() const
{
  return _intervalBetweenRounds;
}

int QCameraWriter::num_saved_frames() const
{
  return _numSavedFrames;
}

int QCameraWriter::num_dropped_frames() const
{
  return _numDroppedFrames;
}

double QCameraWriter::capture_duration() const
{
  return _captureDuration;
}

int QCameraWriter::round() const
{
  return _round;
}

QCameraWriter::State QCameraWriter::state() const
{
  return _currentState;
}

void QCameraWriter::setState(State state)
{
  if( _currentState != state ) {

    _currentState = state;

    Q_EMIT stateChanged();
  }
}

void QCameraWriter::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  Q_EMIT stateChanged();
}

void QCameraWriter::start()
{
  c_unique_lock lock(_mtx);

  if( _currentState == State::Idle ) {

    setState(State::Starting);

    std::thread(&ThisClass::writerThreadProc,
        this).detach();
  }
}

void QCameraWriter::stop()
{
  c_unique_lock lock(_mtx);

  if( _currentState == State::Active ) {

    setState(State::Stopping);
  }

}

void QCameraWriter::writerThreadProc()
{

  static const auto create_video_writer =
      [](QCameraWriter * _this, const QCameraFrame::sptr &frame) -> c_video_frame_writer* {

        c_video_frame_writer * writer = nullptr;

        switch (_this->_outputFormat) {
          case QCameraWriter::FORMAT::SER: {

            _this->_outputFileName =
                qsprintf("%s/%s%s%s.ser",
                    _this->_outputDirectory.toUtf8().constData(),
                    _this->_filenamePrefix.toUtf8().constData(),
                    getCurrentDateTimeString().toUtf8().constData(),
                    _this->_filenameSuffix.toUtf8().constData());

            c_ser_file_writer * w = new c_ser_file_writer();

            if( w->create(_this->_outputFileName, frame->image().size(), frame->colorid(), frame->bpp()) ) {
              writer = w;
            }
            else {
              CF_ERROR("c_ser_file_writer::create('%s') fails",
                  _this->_outputFileName.toUtf8().constData());
              delete w;
            }

            break;
          }

          case QCameraWriter::FORMAT::AVI: {

            if ( _this->_enableSplitStereoStream ) {

              _this->_outputFileName =
                  qsprintf("%s/%s%s%s",
                      _this->_outputDirectory.toUtf8().constData(),
                      _this->_filenamePrefix.toUtf8().constData(),
                      getCurrentDateTimeString().toUtf8().constData(),
                      _this->_filenameSuffix.toUtf8().constData());

              //  _this->output_file_name_ =
              //    QString("%1/%2") .arg(_this->output_directoty_).arg(getCurrentDateTimeString());

              c_avi_stereo_writer * w = new c_avi_stereo_writer();

              const bool fOk = w->create(_this->_outputFileName,
                  _this->_ffmpegOptions,
                  frame->image().size(),
                  _this->_stereoStreamOptions.layout_type,
                  _this->_stereoStreamOptions.swap_cameras,
                  _this->_stereoStreamOptions.downscale_panes,
                  frame->colorid() );

              if ( fOk ) {
                writer = w;
              }
              else {
                CF_ERROR("c_avi_stereo_writer::create('%s') fails",
                    _this->_outputFileName.toUtf8().constData());
                delete w;
              }
            }
            else {

              _this->_outputFileName =
                  qsprintf("%s/%s%s%s.avi",
                      _this->_outputDirectory.toUtf8().constData(),
                      _this->_filenamePrefix.toUtf8().constData(),
                      getCurrentDateTimeString().toUtf8().constData(),
                      _this->_filenameSuffix.toUtf8().constData());


              // _this->output_file_name_ =
              //    QString("%1/%2.avi") .arg(_this->output_directoty_).arg(getCurrentDateTimeString());

              c_avi_file_writer * w = new c_avi_file_writer();

              bool fOk = w->create(_this->_outputFileName,
                  _this->_ffmpegOptions,
                  frame->image().size(),
                  frame->colorid(),
                  frame->bpp());

              if ( fOk ) {
                writer = w;
              }
              else {
                CF_ERROR("c_avi_file_writer::create('%s') fails",
                    _this->_outputFileName.toUtf8().constData());
                delete w;
              }
            }

            break;
          }
          default:
            CF_ERROR("Invalid output format %d requested", _this->_outputFormat);
            break;
        }

        return writer;
      };



  c_unique_lock lock(_mtx);

  QImagingCamera::sptr camera = this->_camera;
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

  QImagingCamera *c = camera.get();
  std::condition_variable_any &condvar = c->condvar();

  const auto wakeup_condition =
      [this, c]() -> bool {
        if ( _currentState != State::Active ) {
          return true;
        }
        if ( c->state() != QImagingCamera::State_started ) {
          return true;
        }
        if ( !c->deque().empty() && c->deque().back()->index() > _lastIndex ) {
          return true;
        }
        return false;
      };

  for( _round = 0; _round < _numRounds ; ++_round ) {


    if ( _enableSplitStereoStream ) {

    }

    c_video_frame_writer *writer = nullptr;

    _lastIndex = -1;
    _numSavedFrames = 0;
    _numDroppedFrames = 0;

    auto start_time = std::chrono::system_clock::now();

    std::chrono::system_clock::time_point start_ts, last_ts;
    //double start_ts = 0;
    //double last_ts = 0;

    int start_index = -1;
    _captureDuration = 0;

    const double max_capture_duration =
        _captureLimits.type == c_capture_limits::ByTime ?
            _captureLimits.value :
            -1;

    const std::string camera_name = camera->name().toStdString();
    const std::string camera_parameters = camera->parameters().toStdString();

    Q_EMIT statusUpdate();

    c_text_file_writer text;
    QString captureStartTime;
    QString captureEndTime;

    while (_currentState == State::Active) {

      lock.unlock();

      QImagingCamera::shared_lock slock(camera->mutex());

      const auto now = std::chrono::system_clock::now();

      if( std::chrono::duration_cast<std::chrono::seconds>(now - start_time) >= std::chrono::seconds(1) ) {
        start_time = now;
        Q_EMIT statusUpdate();
      }

      const auto timeout = std::chrono::milliseconds(100);
      condvar.wait_until(slock, now + timeout, wakeup_condition);

      lock.lock();

      if( _currentState != State::Active ) {
        break;
      }

      if( c->state() != QImagingCamera::State_started ) {
        break;
      }

      const std::deque<QCameraFrame::sptr> &deque = c->deque();

      bool fok = true;

      if ( !deque.empty() ) {

        // Skip early buffered frames as they could be captured at different exposure / gain parameters
        if( _lastIndex < 0 ) {
          _lastIndex = deque.back()->index();
        }

        for( const QCameraFrame::sptr &frame : deque ) {
          if( frame->index() > _lastIndex ) {

            if( !writer ) {

              captureStartTime = getHumanReadableCurrentDateTimeString();
              if( _outputDirectory.isEmpty() ) {
                _outputDirectory =
                    "./capture";
              }

              if( !create_path(_outputDirectory.toStdString()) ) {
                CF_ERROR("create_path('%s') fails: %s",
                    _outputDirectory.toStdString().c_str(),
                    strerror(errno));
                fok = false;
                break;
              }

              if( !(writer = create_video_writer(this, frame)) ) {
                CF_ERROR("create_video_writer() fails");
                fok = false;
                break;
              }

              if( !text.open(ssprintf("%s.txt", _outputFileName.toStdString().c_str())) ) {
                CF_ERROR("text.open('%s.txt') fails",
                    _outputFileName.toStdString().c_str());
              }
            }

            _lastIndex = frame->index();
            if( start_index < 0 ) {
              start_index = _lastIndex;
            }

            //last_ts = frame->ts();
            last_ts = std::chrono::system_clock::now();
            if( !_numSavedFrames ) {
              start_ts = last_ts;
            }

            if( !(fok = writer->write(frame)) ) {
              CF_ERROR("writer->write('%s') fails",
                  _outputFileName.toStdString().c_str());
              break;
            }

            _numDroppedFrames = _lastIndex - start_index - _numSavedFrames;
            ++_numSavedFrames;
            _captureDuration = std::chrono::duration_cast<std::chrono::seconds>(last_ts - start_ts).count();
                //last_ts - start_ts;
          }
        }
      }

      if( !fok ) {
        break;
      }

      if( _captureLimits.value > 0 ) {

        if( _captureLimits.type == c_capture_limits::ByTime ) {
          if( _captureDuration >= max_capture_duration ) {
            CF_DEBUG("capture_duration_=%g / %g", _captureDuration, max_capture_duration);
            break;
          }
        }
        else if( _captureLimits.type == c_capture_limits::ByNumberOfFrames ) {
          if( _numSavedFrames >= _captureLimits.value ) {
            break;
          }
        }
      }
    }

    captureEndTime =
        getHumanReadableCurrentDateTimeString();

    if ( text.is_open() ) {

      fprintf(text.fp, "Round    = %d // Current capture round\n", _round);
      fprintf(text.fp, "Rounds   = %d // Total capture rounds\n", (int)_numRounds);
      fprintf(text.fp, "Start    = %s // Capture start time YYYY-MM-DD hh mm ss\n", captureStartTime.toUtf8().constData());
      fprintf(text.fp, "End      = %s // Capture end time YYYY-MM-DD hh mm ss\n", captureEndTime.toUtf8().constData());
      fprintf(text.fp, "Limit    = %s // Capture limit\n", toQString(_captureLimits).toUtf8().constData());
      fprintf(text.fp, "Camera   = %s\n", camera_name.c_str());
      fprintf(text.fp, "Frames   = %d // Number of recorded frames\n", _numSavedFrames);
      fprintf(text.fp, "%s\n", camera_parameters.c_str());
      fprintf(text.fp, "\n");

      text.close();
    }

    if( writer ) {
      delete writer;
    }

    Q_EMIT statusUpdate();

    for ( int i = 0; i < _intervalBetweenRounds; ++i ) {
      if ( _currentState != State::Active || c->state() != QImagingCamera::State_started ) {
        break;
      }
      sleep(1);
    }

    if ( _currentState != State::Active || c->state() != QImagingCamera::State_started ) {
      break;
    }

  }

  setState(State::Idle);
  Q_EMIT statusUpdate();
}

} /* namespace serimager
 */
