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
    public QSettingsWidget
{
public:
  typedef QDisplayVideoWriterOptions ThisClass;
  typedef QSettingsWidget Base;

  QDisplayVideoWriterOptions(QWidget * parent = nullptr);

  void setVideoWriter(QDisplayVideoWriter * writer);
  QDisplayVideoWriter * videoWriter() const;

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;

protected:
  QDisplayVideoWriter * videoWriter_ = nullptr;
  QBrowsePathCombo * outputPath_ctl = nullptr;
  QFFmpegOptionsControl * ffoptions_ctl = nullptr;
  //QLineEditBox * ffoptions_ctl = nullptr;
  QLineEditBox * outputFilenamePrefix_ctl = nullptr;
  QLineEditBox * outputFilenameSuffix_ctl = nullptr;
  QCheckBox * writeViewPort_ctl = nullptr;
};


class QDisplayVideoWriterOptionsDialogBox:
    public QDialog
{
  Q_OBJECT;

public:
  typedef QDisplayVideoWriterOptionsDialogBox ThisClass;
  typedef QDialog Base;

  QDisplayVideoWriterOptionsDialogBox(const QString & title, QWidget * parent = nullptr);
  QDisplayVideoWriterOptionsDialogBox(QWidget * parent = nullptr);

  void setVideoWriter(QDisplayVideoWriter * writer);
  QDisplayVideoWriter * videoWriter() const;

  void loadParameters();

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent *e) override;
  void hideEvent(QHideEvent *e) override;

protected:
  QDisplayVideoWriterOptions * options_ctl = nullptr;
};

QToolButton* createDisplayVideoWriterOptionsToolButton(QDisplayVideoWriter * writer, QWidget * parent = nullptr);


#endif /* __QDisplayVideoWriterOptions_h__ */
