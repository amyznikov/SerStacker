/*
 * QDisplayVideoWriter.cc
 *
 *  Created on: May 25, 2023
 *      Author: amyznikov
 */

#include "QDisplayVideoWriter.h"
#include <gui/widgets/qsprintf.h>
#include <core/debug.h>

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

  return qsprintf("%0.4d%0.2d%0.2d_%0.2d%0.2d%0.2d_GMT",
      year, month, day, hour, min, sec);
}


QDisplayVideoWriter::QDisplayVideoWriter(QObject * parent) :
  Base(parent)
{
}

const QString & QDisplayVideoWriter::outputPath() const
{
  return _outputPath;
}

void QDisplayVideoWriter::setOutputPath(const QString & v)
{
  _outputPath = v;
}

const QString & QDisplayVideoWriter::ffoptions() const
{
  return _ffoptions;
}

void QDisplayVideoWriter::setFfoptions(const QString & v)
{
  _ffoptions = v;
}

const QString & QDisplayVideoWriter::outputFilenamePrefix() const
{
  return _outputFilenamePrefix;
}

void QDisplayVideoWriter::setOutputFilenamePrefix(const QString & v)
{
  _outputFilenamePrefix = v;
}

const QString & QDisplayVideoWriter::outputFilenameSuffix() const
{
  return _outputFilenameSuffix;
}

void QDisplayVideoWriter::setOutputFilenameSuffix(const QString & v)
{
  _outputFilenameSuffix = v;
}

void QDisplayVideoWriter::setWriteViewPort(bool v)
{
  _writeViewPort = v;
}

bool QDisplayVideoWriter::writeViewPort() const
{
  return _writeViewPort;
}

const QString & QDisplayVideoWriter::outputPathFileName() const
{
  return _outputPathFileName;
}

bool QDisplayVideoWriter::start()
{
  _started = true;

  if ( !_paused ) {

    if( _ffmpeg.is_open() ) {
      stop();
    }

    _nbframes = 0;
    _frameSize = cv::Size(-1, -1);
    _channels = 0;
  }

  Q_EMIT stateChanged();

  return true;
}

void QDisplayVideoWriter::stop()
{
  if ( _ffmpeg.is_open() ) {
    _ffmpeg.close();
    CF_DEBUG("Closed video file '%s' ", _ffmpeg.filename().c_str());
  }

  _started = false;

  Q_EMIT stateChanged();
}


void QDisplayVideoWriter::set_paused(bool v)
{
  _paused = v;
  Q_EMIT stateChanged();
}

bool QDisplayVideoWriter::started() const
{
  return _started;
}

bool QDisplayVideoWriter::paused() const
{
  return _paused;
}


int QDisplayVideoWriter::nbframes() const
{
  return _nbframes;
}

bool QDisplayVideoWriter::write(const cv::Mat & _frame)
{
  if( !_started ) {
    CF_ERROR("ERROR: Video record was not stared");
    return false;
  }

  cv::Mat frame;

  if( _nbframes > 0 && _frame.cols >= _frameSize.width && _frame.rows >= _frameSize.height ) {
    frame = _frame(cv::Rect(0, 0, _frameSize.width, _frameSize.height));
  }
  else {
    frame = _frame;
  }

  if( _nbframes > 0 && (_frameSize != frame.size() || _channels != frame.channels()) ) {

    stop();

    if( !start() ) {
      CF_ERROR("Restart video record fails");
      return false;
    }

  }

  if( !_ffmpeg.is_open() ) {

    if( _outputPath.isEmpty() ) {
      _outputPath = "./curremtDisplay";
    }

    _outputPathFileName =
        qsprintf("%s/%s%s%s.avi",
            _outputPath.toUtf8().constData(),
            _outputFilenamePrefix.toUtf8().constData(),
            getCurrentDateTimeString().toUtf8().constData(),
            _outputFilenameSuffix.toUtf8().constData());

    _frameSize = frame.size();
    _channels = frame.channels();

    if( !_ffmpeg.open(_outputPathFileName.toStdString(), _frameSize, _channels > 1, _ffoptions.toStdString()) ) {
      CF_ERROR("ffmpeg_.open('%s') fails for frameSize_=%dx%d channels=%d options='%s'",
          _outputPathFileName.toUtf8().constData(),
          _frameSize.width, _frameSize.height,
          frame.channels(),
          _ffoptions.toUtf8().constData());

      stop();
      return false;
    }

    CF_DEBUG("Created video file '%s' ", _ffmpeg.filename().c_str());
  }

  if( !_ffmpeg.write(frame, _nbframes++) ) {
    CF_ERROR("ffmpeg_.write() fails");
    stop();
    return false;
  }

  return true;
}

void QDisplayVideoWriter::loadSettings(const QString & prefix)
{
  QSettings settings;
  loadSettings(settings, prefix);
}

void QDisplayVideoWriter::saveSettings(const QString & prefix) const
{
  QSettings settings;
  saveSettings(settings, prefix);
}

void QDisplayVideoWriter::loadSettings(const QSettings & settings, const QString & prefix)
{
  const QString PREFIX = prefix.isEmpty() ? "QDisplayVideoWriter" : prefix;
  const auto PARAM =
      [PREFIX](const QString & name) {
    return QString("%1/%2").arg(PREFIX).arg(name);
  };

  _outputPath = settings.value(PARAM("outputPath"), _outputPath).toString();
  _ffoptions = settings.value(PARAM("ffoptions"), _ffoptions).toString();
  _outputFilenamePrefix = settings.value(PARAM("outputFilenamePrefix"), _outputFilenamePrefix).toString();
  _outputFilenameSuffix = settings.value(PARAM("outputFilenameSuffix"), _outputFilenameSuffix).toString();
  _writeViewPort = settings.value(PARAM("writeViewPort"), _writeViewPort).toBool();
}

void QDisplayVideoWriter::saveSettings(QSettings & settings, const QString & prefix) const
{
  const QString PREFIX = prefix.isEmpty() ? "QDisplayVideoWriter" : prefix;
  const auto PARAM =
      [PREFIX](const QString & name) {
    return QString("%1/%2").arg(PREFIX).arg(name);
  };

  settings.setValue(PARAM("outputPath"), _outputPath);
  settings.setValue(PARAM("ffoptions"), _ffoptions);
  settings.setValue(PARAM("outputFilenamePrefix"), _outputFilenamePrefix);
  settings.setValue(PARAM("outputFilenameSuffix"), _outputFilenameSuffix);
  settings.setValue(PARAM("writeViewPort"), _writeViewPort);
}



