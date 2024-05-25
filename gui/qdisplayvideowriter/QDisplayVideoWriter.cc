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
  return outputPath_;
}

void QDisplayVideoWriter::setOutputPath(const QString & v)
{
  outputPath_ = v;
  saveParameters();
}

const QString & QDisplayVideoWriter::ffoptions() const
{
  return ffoptions_;
}

void QDisplayVideoWriter::setFfoptions(const QString & v)
{
  ffoptions_ = v;
  saveParameters();
}

const QString & QDisplayVideoWriter::outputFilenamePrefix() const
{
  return outputFilenamePrefix_;
}

void QDisplayVideoWriter::setOutputFilenamePrefix(const QString & v)
{
  outputFilenamePrefix_ = v;
  saveParameters();
}

const QString & QDisplayVideoWriter::outputFilenameSuffix() const
{
  return outputFilenameSuffix_;
}

void QDisplayVideoWriter::setOutputFilenameSuffix(const QString & v)
{
  outputFilenameSuffix_ = v;
  saveParameters();
}

void QDisplayVideoWriter::setWriteViewPort(bool v)
{
  writeViewPort_ = v;
  saveParameters();
}

bool QDisplayVideoWriter::writeViewPort() const
{
  return writeViewPort_;
}

const QString & QDisplayVideoWriter::outputPathFileName() const
{
  return outputPathFileName_;
}

bool QDisplayVideoWriter::start()
{
  started_ = true;

  if ( !paused_ ) {

    if( ffmpeg_.is_open() ) {
      stop();
    }

    nbframes_ = 0;
    frameSize_ = cv::Size(-1, -1);
    channels_ = 0;
  }

  Q_EMIT stateChanged();

  return true;
}

void QDisplayVideoWriter::stop()
{
  if ( ffmpeg_.is_open() ) {
    ffmpeg_.close();
    CF_DEBUG("Closed video file '%s' ", ffmpeg_.filename().c_str());
  }

  started_ = false;

  Q_EMIT stateChanged();
}


void QDisplayVideoWriter::set_paused(bool v)
{
  paused_ = v;
  Q_EMIT stateChanged();
}

bool QDisplayVideoWriter::started() const
{
  return started_;
}

bool QDisplayVideoWriter::paused() const
{
  return paused_;
}


int QDisplayVideoWriter::nbframes() const
{
  return nbframes_;
}

bool QDisplayVideoWriter::write(const cv::Mat & _frame)
{
  if( !started_ ) {
    CF_ERROR("ERROR: Video record was not stared");
    return false;
  }

  cv::Mat frame;

  if( nbframes_ > 0 && _frame.cols >= frameSize_.width && _frame.rows >= frameSize_.height ) {
    frame = _frame(cv::Rect(0, 0, frameSize_.width, frameSize_.height));
  }
  else {
    frame = _frame;
  }

  if( nbframes_ > 0 && (frameSize_ != frame.size() || channels_ != frame.channels()) ) {

    stop();

    if( !start() ) {
      CF_ERROR("Restart video record fails");
      return false;
    }

  }

  if( !ffmpeg_.is_open() ) {

    if( outputPath_.isEmpty() ) {
      outputPath_ = "./curremtDisplay";
    }

    outputPathFileName_ =
        qsprintf("%s/%s%s%s.avi",
            outputPath_.toUtf8().constData(),
            outputFilenamePrefix_.toUtf8().constData(),
            getCurrentDateTimeString().toUtf8().constData(),
            outputFilenameSuffix_.toUtf8().constData());

    frameSize_ = frame.size();
    channels_ = frame.channels();

    if( !ffmpeg_.open(outputPathFileName_.toStdString(), frameSize_, channels_ > 1, ffoptions_.toStdString()) ) {
      CF_ERROR("ffmpeg_.open('%s') fails for frameSize_=%dx%d channels=%d options='%s'",
          outputPathFileName_.toUtf8().constData(),
          frameSize_.width, frameSize_.height,
          frame.channels(),
          ffoptions_.toUtf8().constData());

      stop();
      return false;
    }

    CF_DEBUG("Created video file '%s' ", ffmpeg_.filename().c_str());
  }

  if( !ffmpeg_.write(frame, nbframes_++) ) {
    CF_ERROR("ffmpeg_.write() fails");
    stop();
    return false;
  }

  return true;
}

void QDisplayVideoWriter::loadParameters()
{
  QSettings settings;

  outputPath_ = settings.value("QDisplayVideoWriter/outputPath", outputPath_).toString();
  ffoptions_ = settings.value("QDisplayVideoWriter/ffoptions", ffoptions_).toString();
  outputFilenamePrefix_ = settings.value("QDisplayVideoWriter/outputFilenamePrefix", outputFilenamePrefix_).toString();
  outputFilenameSuffix_ = settings.value("QDisplayVideoWriter/outputFilenameSuffix", outputFilenameSuffix_).toString();
  writeViewPort_ = settings.value("QDisplayVideoWriter/writeViewPort", writeViewPort_).toBool();

}

void QDisplayVideoWriter::saveParameters() const
{
  QSettings settings;

  settings.setValue("QDisplayVideoWriter/outputPath", outputPath_);
  settings.setValue("QDisplayVideoWriter/ffoptions", ffoptions_);
  settings.setValue("QDisplayVideoWriter/outputFilenamePrefix", outputFilenamePrefix_);
  settings.setValue("QDisplayVideoWriter/outputFilenameSuffix", outputFilenameSuffix_);
  settings.setValue("QDisplayVideoWriter/writeViewPort", writeViewPort_);
}


