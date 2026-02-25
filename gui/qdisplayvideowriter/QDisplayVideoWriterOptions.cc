/*
 * QDisplayVideoWriterOptions.cc
 *
 *  Created on: May 25, 2023
 *      Author: amyznikov
 */

#include "QDisplayVideoWriterOptions.h"
#include <gui/widgets/style.h>
#include <gui/widgets/createAction.h>
#include <gui/widgets/settings.h>
#include <core/debug.h>

#define ICON_video_start    ":/qdisplayvideowriter/icons/video-start.png"
#define ICON_video_stop1    ":/qdisplayvideowriter/icons/video-stop1.png"
#define ICON_video_stop2    ":/qdisplayvideowriter/icons/video-stop2.png"
#define ICON_video_options  ":/qdisplayvideowriter/icons/video_options.png"

///////////////////////////////////////////////////////////////////////////////////////////////////

QDisplayVideoWriterOptions::QDisplayVideoWriterOptions(QWidget * parent) :
    Base(parent)
{
  Q_INIT_RESOURCE(qdisplayvideowriter_resources);

  outputPath_ctl =
      add_browse_for_path("Output path:", "",
          QFileDialog::AcceptMode::AcceptSave,
          QFileDialog::FileMode::Directory,
          [this](const QString & s) {
            if ( _opts ) {
              _opts->setOutputPath(s);
            }
          },
          [this](QString * s) {
            if ( _opts ) {
              * s = _opts->outputPath();
              return true;
            }
            return false;
          });

  ffoptions_ctl =
      add_ffmpeg_options_control("ffmpeg opts:", "Options for ffmpeg AVI writer",
          [this](const QString & s) {
            if ( _opts ) {
              _opts->setFfoptions(s);
            }
          },
          [this](QString * s) {
            if ( _opts ) {
              * s = _opts->ffoptions();
              return true;
            }
            return false;
          });

  outputFilenamePrefix_ctl =
      add_textbox("Prefix:", "Optional prefix for output file name",
          [this](const QString & s) {
            if ( _opts ) {
              _opts->setOutputFilenamePrefix(s);
            }
          },
          [this](QString * s) {
            if ( _opts ) {
              *s = _opts->outputFilenamePrefix();
              return true;
            }
            return false;
          });

  outputFilenameSuffix_ctl =
      add_textbox("Suffix:", "Optional suffix for output file name",
          [this](const QString & s) {
            if ( _opts ) {
              _opts->setOutputFilenameSuffix(s);
            }
          },
          [this](QString * s) {
            if ( _opts ) {
              *s = _opts->outputFilenameSuffix();
              return true;
            }
            return false;
          });

  writeViewPort_ctl =
      add_checkbox("Write Viewport",
          "Set checked to write image viewer viewport instead of raw display image",
          [this](bool checked) {
            if ( _opts ) {
              _opts->setWriteViewPort(checked);
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->writeViewPort();
              return true;
            }
            return false;
          });

  updateControls();
}

void QDisplayVideoWriterOptions::setVideoWriter(QDisplayVideoWriter * writer)
{
  if( _opts ) {
    _opts->disconnect(this);
  }

  if( (_opts = writer) ) {
  }

  updateControls();
}

QDisplayVideoWriter* QDisplayVideoWriterOptions::videoWriter() const
{
  return _opts;
}

void QDisplayVideoWriterOptions::onload(const QSettings & settings, const QString & prefix)
{
  if( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "QDisplayVideoWriter" : prefix;

    QString outputPath = _opts->outputPath();
    if( load_parameter(settings, PREFIX, "outputPath", &outputPath) ) {
      _opts->setOutputPath(outputPath);
    }

    QString ffoptions = _opts->ffoptions();
    if( load_parameter(settings, PREFIX, "ffoptions", &ffoptions) ) {
      _opts->setFfoptions(ffoptions);
    }

    QString outputFilenamePrefix = _opts->outputFilenamePrefix();
    if( load_parameter(settings, PREFIX, "outputFilenamePrefix", &outputFilenamePrefix) ) {
      _opts->setOutputFilenamePrefix(outputFilenamePrefix);
    }

    QString outputFilenameSuffix = _opts->outputFilenameSuffix();
    if( load_parameter(settings, PREFIX, "outputFilenameSuffix", &outputFilenameSuffix) ) {
      _opts->setOutputFilenameSuffix(outputFilenameSuffix);
    }
  }
}

void QDisplayVideoWriterOptions::onsave(QSettings & settings, const QString & prefix)
{
  if( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "QDisplayVideoWriter" : prefix;

    save_parameter(settings, PREFIX, "outputPath", _opts->outputPath());
    save_parameter(settings, PREFIX, "ffoptions", _opts->ffoptions());
    save_parameter(settings, PREFIX, "outputFilenamePrefix", _opts->outputFilenamePrefix());
    save_parameter(settings, PREFIX, "outputFilenameSuffix", _opts->outputFilenameSuffix());
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////

QDisplayVideoWriterOptionsDialogBox::QDisplayVideoWriterOptionsDialogBox(QWidget * parent) :
    ThisClass("Video Writer Options", parent)
{
}

QDisplayVideoWriterOptionsDialogBox::QDisplayVideoWriterOptionsDialogBox(const QString & title, QWidget * parent) :
    Base(title, parent)
{
  setWindowIcon(getIcon(ICON_video_options));
}

void QDisplayVideoWriterOptionsDialogBox::setVideoWriter(QDisplayVideoWriter * writer)
{
  _settings->setVideoWriter(writer);
}

QDisplayVideoWriter* QDisplayVideoWriterOptionsDialogBox::videoWriter() const
{
  return _settings->videoWriter();
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
          dlgbox->loadSettings();
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
