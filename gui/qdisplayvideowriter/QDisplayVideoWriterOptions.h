/*
 * QDisplayVideoWriterOptions.h
 *
 *  Created on: May 25, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QDisplayVideoWriterOptions_h__
#define __QDisplayVideoWriterOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QDisplayVideoWriter.h"


class QDisplayVideoWriterOptions:
    public QSettingsWidgetTemplate<QDisplayVideoWriter>
{
public:
  typedef QDisplayVideoWriterOptions ThisClass;
  typedef QSettingsWidgetTemplate<QDisplayVideoWriter> Base;

  QDisplayVideoWriterOptions(QWidget * parent = nullptr);

  void setVideoWriter(QDisplayVideoWriter * writer);
  QDisplayVideoWriter * videoWriter() const;

protected:
  void onload(const QSettings & settings, const QString & prefix = "") override;
  void onsave(QSettings & settings, const QString & prefix = "") override;

protected:
  QBrowsePathCombo * outputPath_ctl = nullptr;
  QFFmpegOptionsControl * ffoptions_ctl = nullptr;
  QLineEditBox * outputFilenamePrefix_ctl = nullptr;
  QLineEditBox * outputFilenameSuffix_ctl = nullptr;
  QCheckBox * writeViewPort_ctl = nullptr;
};


class QDisplayVideoWriterOptionsDialogBox:
    public QSettingsDialogBoxTemplate<QDisplayVideoWriterOptions>
{
  Q_OBJECT;

public:
  typedef QDisplayVideoWriterOptionsDialogBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QDisplayVideoWriterOptions> Base;

  QDisplayVideoWriterOptionsDialogBox(const QString & title, QWidget * parent = nullptr);
  QDisplayVideoWriterOptionsDialogBox(QWidget * parent = nullptr);

  void setVideoWriter(QDisplayVideoWriter * writer);
  QDisplayVideoWriter * videoWriter() const;
};

QToolButton* createDisplayVideoWriterOptionsToolButton(QDisplayVideoWriter * writer, QWidget * parent = nullptr);


#endif /* __QDisplayVideoWriterOptions_h__ */
