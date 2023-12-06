/*
 * QDisplayVideoWriter.h
 *
 *  Created on: May 25, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QDisplayVideoWriter_h__
#define __QDisplayVideoWriter_h__

#include <QtCore/QtCore>
#include <core/io/c_ffmpeg_file.h>

class QDisplayVideoWriter :
    public QObject
{
  Q_OBJECT;
public:
  typedef QDisplayVideoWriter ThisClass;
  typedef QObject Base;

  QDisplayVideoWriter(QObject * parent = nullptr);

  void setOutputPath(const QString & );
  const QString & outputPath() const;

  void setFfoptions(const QString & );
  const QString & ffoptions() const;

  void setOutputFilenamePrefix(const QString & );
  const QString & outputFilenamePrefix() const;

  void setOutputFilenameSuffix(const QString & );
  const QString & outputFilenameSuffix() const;

  void setWriteViewPort(bool v);
  bool writeViewPort() const;

  const QString & outputPathFileName() const;


  bool start();
  void stop();
  void pause();

  bool started() const;
  bool paused() const;

  bool write(const cv::Mat & frame);
  int nbframes() const;

  void loadParameters();
  void saveParameters() const;

Q_SIGNALS:
  void stateChanged();

protected:
  c_ffmpeg_writer ffmpeg_;
  QString outputPath_;
  QString ffoptions_ = "-c huffyuv -r 10 -f avi";
  QString outputFilenamePrefix_;
  QString outputFilenameSuffix_;
  QString outputPathFileName_;
  bool started_ = false;
  bool paused_ = false;
  bool writeViewPort_ = false;
  int nbframes_ = 0;
  cv::Size frameSize_;
  int channels_ = 0;
};

#endif /* __QDisplayVideoWriter_h__ */
