/*
 * QCameraWriter.h
 *
 *  Created on: Dec 29, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCameraWriter_h__
#define __QCameraWriter_h__

#include <QtCore/QtCore>
#include "QImagingCamera.h"
#include <core/proc/stereo/stereo_stream.h>

namespace serimager {

struct c_capture_limits
{
public:
  enum TYPE {
    ByTime,
    ByNumberOfFrames
  };

  TYPE type = ByTime;
  int value = -1; // unlimited
};


class QCameraWriter:
    public QObject
{
  Q_OBJECT;

public:
  typedef QCameraWriter ThisClass;
  typedef QObject Base;
  typedef std::unique_lock<std::mutex> c_unique_lock;


  enum FORMAT {
    SER,
    AVI,
    IMAGES
  };

  enum State {
    Idle,
    Starting,
    Active,
    Stopping
  };

  QCameraWriter(QObject * parent = nullptr);

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr & camera() const;

  void setOutputDirectoty(const QString & path);
  const QString & outputDirectoty() const;

  void setOutputFormat(FORMAT v);
  FORMAT outputFormat() const;

  void setFFmpegOptions(const QString & opts);
  const QString & ffmpegOptions() const;

  void setFilenamePrefix(const QString & );
  const QString & filenamePrefix() const;

  void setFilenameSuffix(const QString & );
  const QString & filenameSuffix() const;

  void setCaptureLimits(const c_capture_limits & limits);
  const c_capture_limits & captureLimits() const;

  void set_enable_split_stereo_stream(bool v);
  bool enable_split_stereo_stream() const;

  c_stereo_stream_options & stereo_stream_options();
  const c_stereo_stream_options & stereo_stream_options() const;

  void setNumRounds(int v);
  int numRounds() const;

  void setIntervalBetweenRounds(int v);
  int intervalBetweenRounds() const;



  State state() const;
  int num_saved_frames() const;
  int num_dropped_frames() const;
  double capture_duration() const;
  int round() const;

  void start();
  void stop();

Q_SIGNALS:
  void stateChanged();
  void statusUpdate();

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

protected:
  void setState(State state);
  void writerThreadProc();


protected:
  std::mutex _mtx;
  QImagingCamera::sptr _camera;
  c_capture_limits _capture_limits;
  QString _output_directoty;
  QString _output_file_name;
  QString _filename_prefix;
  QString _filename_suffix;

  FORMAT _output_format = FORMAT::SER;
  QString _ffmpeg_options = "-r 10 -c rawvideo -pix_fmt bgr24"; // "-c huffyuv -r 100 -f avi";

  bool _enable_split_stereo_stream = false;
  c_stereo_stream_options _stereo_stream_options;

  std::atomic<int> _numRounds = 1;
  std::atomic<int> _interval_between_rounds = 0;

  State _current_state = State::Idle;
  int _last_index = -1;
  int _num_saved_frames = 0;
  int _num_dropped_frames = 0;
  int _round = 0;
  double _capture_duration = 0;
};


QString toQString(const c_capture_limits & limits);

} /* namespace serimager */

Q_DECLARE_METATYPE(serimager::c_capture_limits);
Q_DECLARE_METATYPE(serimager::QCameraWriter::FORMAT);
Q_DECLARE_METATYPE(serimager::QCameraWriter::State);


#endif /* __QCameraWriter_h__ */
