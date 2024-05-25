/*
 * QDisplayVideoWriterOptions.cc
 *
 *  Created on: May 25, 2023
 *      Author: amyznikov
 */

#include "QDisplayVideoWriterOptions.h"
#include <gui/widgets/style.h>
#include <gui/widgets/createAction.h>
#include <core/debug.h>

#define ICON_video_start    ":/qdisplayvideowriter/icons/video-start.png"
#define ICON_video_stop1    ":/qdisplayvideowriter/icons/video-stop1.png"
#define ICON_video_stop2    ":/qdisplayvideowriter/icons/video-stop2.png"
#define ICON_video_options  ":/qdisplayvideowriter/icons/video_options.png"

///////////////////////////////////////////////////////////////////////////////////////////////////

QDisplayVideoWriterOptions::QDisplayVideoWriterOptions(QWidget * parent) :
    Base("QDisplayVideoWriterOptions", parent)
{
  Q_INIT_RESOURCE(qdisplayvideowriter_resources);

  outputPath_ctl =
      add_browse_for_path("Output path:", "",
          QFileDialog::AcceptMode::AcceptSave,
          QFileDialog::FileMode::Directory,
          [this](const QString & s) {
            if ( videoWriter_ ) {
              videoWriter_->setOutputPath(s);
              save_parameter(PREFIX, "outputPath", videoWriter_->outputPath());
            }
          },
          [this](QString * s) {
            if ( videoWriter_ ) {
              * s = videoWriter_->outputPath();
              return true;
            }
            return false;
          });

  ffoptions_ctl =
      add_ffmpeg_options_control("ffmpeg opts:", "Options for ffmpeg AVI writer",
          [this](const QString & s) {
            if ( videoWriter_ ) {
              videoWriter_->setFfoptions(s);
              save_parameter(PREFIX, "ffoptions", videoWriter_->ffoptions());
            }
          },
          [this](QString * s) {
            if ( videoWriter_ ) {
              * s = videoWriter_->ffoptions();
              return true;
            }
            return false;
          });

  outputFilenamePrefix_ctl =
      add_textbox("Prefix:", "Optional prefix for output file name",
          [this](const QString & s) {
            if ( videoWriter_ ) {
              videoWriter_->setOutputFilenamePrefix(s);
              save_parameter(PREFIX, "outputFilenamePrefix", videoWriter_->outputFilenamePrefix());
            }
          },
          [this](QString * s) {
            if ( videoWriter_ ) {
              *s = videoWriter_->outputFilenamePrefix();
              return true;
            }
            return false;
          });

  outputFilenameSuffix_ctl =
      add_textbox("Suffix:", "Optional suffix for output file name",
          [this](const QString & s) {
            if ( videoWriter_ ) {
              videoWriter_->setOutputFilenameSuffix(s);
              save_parameter(PREFIX, "outputFilenameSuffix", videoWriter_->outputFilenameSuffix());
            }
          },
          [this](QString * s) {
            if ( videoWriter_ ) {
              *s = videoWriter_->outputFilenameSuffix();
              return true;
            }
            return false;
          });

  writeViewPort_ctl =
      add_checkbox("Write Viewport",
          "Set checked to write image viewer viewport instead of raw display image",
          [this](bool checked) {
            if ( videoWriter_ ) {
              videoWriter_->setWriteViewPort(checked);
              save_parameter(PREFIX, "writeViewPort", videoWriter_->writeViewPort());
            }
          },
          [this](bool * checked) {
            if ( videoWriter_ ) {
              * checked = videoWriter_->writeViewPort();
              return true;
            }
            return false;
          });

  updateControls();
}

void QDisplayVideoWriterOptions::setVideoWriter(QDisplayVideoWriter * writer)
{
  if( videoWriter_ ) {
    videoWriter_->disconnect(this);
  }

  if( (videoWriter_ = writer) ) {
  }

  updateControls();
}

QDisplayVideoWriter* QDisplayVideoWriterOptions::videoWriter() const
{
  return videoWriter_;
}

void QDisplayVideoWriterOptions::onupdatecontrols()
{
  if( !videoWriter_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

void QDisplayVideoWriterOptions::onload(QSettings & settings)
{
  if( videoWriter_ ) {

    QString outputPath = videoWriter_->outputPath();
    if( load_parameter(settings, PREFIX, "outputPath", &outputPath) ) {
      videoWriter_->setOutputPath(outputPath);
    }

    QString ffoptions = videoWriter_->ffoptions();
    if( load_parameter(settings, PREFIX, "ffoptions", &ffoptions) ) {
      videoWriter_->setFfoptions(ffoptions);
    }

    QString outputFilenamePrefix = videoWriter_->outputFilenamePrefix();
    if( load_parameter(settings, PREFIX, "outputFilenamePrefix", &outputFilenamePrefix) ) {
      videoWriter_->setOutputFilenamePrefix(outputFilenamePrefix);
    }

    QString outputFilenameSuffix = videoWriter_->outputFilenameSuffix();
    if( load_parameter(settings, PREFIX, "outputFilenameSuffix", &outputFilenameSuffix) ) {
      videoWriter_->setOutputFilenameSuffix(outputFilenameSuffix);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QDisplayVideoWriterOptionsDialogBox::QDisplayVideoWriterOptionsDialogBox(QWidget * parent) :
    ThisClass("Video Writer Options", parent)
{
}

QDisplayVideoWriterOptionsDialogBox::QDisplayVideoWriterOptionsDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  QVBoxLayout *vbox = new QVBoxLayout(this);
  vbox->addWidget(options_ctl = new QDisplayVideoWriterOptions(this));
  setWindowTitle(title);
  setWindowIcon(getIcon(ICON_video_options));
}

void QDisplayVideoWriterOptionsDialogBox::setVideoWriter(QDisplayVideoWriter * writer)
{
  options_ctl->setVideoWriter(writer);
}

QDisplayVideoWriter* QDisplayVideoWriterOptionsDialogBox::videoWriter() const
{
  return options_ctl->videoWriter();
}

void QDisplayVideoWriterOptionsDialogBox::loadParameters()
{
  options_ctl->loadParameters();
}

void QDisplayVideoWriterOptionsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QDisplayVideoWriterOptionsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QToolButton* createDisplayVideoWriterOptionsToolButton(QDisplayVideoWriter * writer, QWidget * parent)
{
  Q_INIT_RESOURCE(qdisplayvideowriter_resources);

  QToolButton *tb = new QToolButton();
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(getIcon(ICON_video_start));
  tb->setText("Start");
  tb->setToolTip("Start / Stop recording display video");
  tb->setCheckable(true);
  tb->setPopupMode(QToolButton::ToolButtonPopupMode::MenuButtonPopup);

  QMenu *menu = new QMenu(tb);

  menu->addAction(createCheckableAction(QIcon(),
      "Pause",
      "Pause / Resume video recording",
      writer->paused(),
      [writer](bool checked) {
        writer->set_paused(checked);
      }));


  menu->addAction(createCheckableAction2(getIcon(ICON_video_options),
      "Video record options...",
      "Show / Hide video record options dialog box",
      false,
      [writer, tb, parent](QAction * action) {

        static QDisplayVideoWriterOptionsDialogBox * dlgbox =
            nullptr;

        const bool checked =
            action->isChecked();

        if ( !checked ) {

          if ( dlgbox ) {
            dlgbox->hide();
          }
        }
        else {

          if ( dlgbox ) {
            dlgbox->setParent(parent ? parent : tb);
          }
          else {
            dlgbox = new QDisplayVideoWriterOptionsDialogBox(parent? parent : tb);

            QObject::connect(dlgbox, &QDisplayVideoWriterOptionsDialogBox::visibilityChanged,
                action, &QAction::setChecked);
          }

          dlgbox->setVideoWriter(writer);
          dlgbox->show();
        }

      }));

  tb->setMenu(menu);

  QObject::connect(tb, &QToolButton::clicked,
      [writer, tb](bool checked) {

        if ( writer->started() ) {
          writer->stop();
        }
        else if ( !writer->start() ) {
          CF_ERROR("writer->start() fails");
        }

        tb->setChecked(writer->started());
      });

  QObject::connect(writer, &QDisplayVideoWriter::stateChanged,
      [writer, tb]() {

        tb->setChecked(writer->started());

        if ( writer->started() ) {
          tb->setIcon(getIcon(ICON_video_stop1));
        }
        else {
          tb->setIcon(getIcon(ICON_video_start));
        }
      });

  return tb;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
