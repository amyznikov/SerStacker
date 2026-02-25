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

  bool started() const;
  bool paused() const;
  void set_paused(bool v);

  bool write(const cv::Mat & frame);
  int nbframes() const;

  void loadSettings(const QString & prefix = "");
  void loadSettings(const QSettings & settings, const QString & prefix = "");
  void saveSettings(const QString & prefix = "") const;
  void saveSettings(QSettings & settings, const QString & prefix = "") const;

Q_SIGNALS:
  void stateChanged();

protected:
  c_ffmpeg_writer _ffmpeg;
  QString _outputPath;
  QString _ffoptions = "-c huffyuv -r 10 -f avi";
  QString _outputFilenamePrefix;
  QString _outputFilenameSuffix;
  QString _outputPathFileName;
  bool _started = false;
  bool _paused = false;
  bool _writeViewPort = false;
  int _nbframes = 0;
  cv::Size _frameSize;
  int _channels = 0;
};

#endif /* __QDisplayVideoWriter_h__ */
