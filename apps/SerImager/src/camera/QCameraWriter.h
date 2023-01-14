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

  void setCaptureLimits(const c_capture_limits & limits);
  const c_capture_limits & captureLimits() const;

  void setRounds(int v);
  int rounds() const;

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
  void statisticsUpdate();

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

protected:
  void setState(State state);
  void writerThreadProc();


protected:
  std::mutex mtx_;
  QImagingCamera::sptr camera_;
  c_capture_limits capture_limits_;
  QString output_directoty_;
  QString output_file_name_;
  FORMAT output_format_ = FORMAT::SER;
  int rounds_ = 1;
  int interval_between_rounds_ = 0;
  State current_state_ = State::Idle;
  int last_index_ = -1;
  int num_saved_frames_ = 0;
  int num_dropped_frames_ = 0;
  int round_ = 0;
  double capture_duration_ = 0;
};


QString toQString(const c_capture_limits & limits);

} /* namespace serimager */

Q_DECLARE_METATYPE(serimager::c_capture_limits);
Q_DECLARE_METATYPE(serimager::QCameraWriter::FORMAT);
Q_DECLARE_METATYPE(serimager::QCameraWriter::State);


#endif /* __QCameraWriter_h__ */
