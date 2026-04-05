/*
 * QFFmpegOptionsControl.cc
 *
 *  Created on: Feb 18, 2024
 *      Author: amyznikov
 */

#include "QFFmpegOptionsControl.h"
#include <gui/widgets/QTextInfoDialogBox.h>
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/createAction.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/debug.h>

QFFmpegOptionsControl::QFFmpegOptionsControl(QWidget * parent) :
  Base(parent)
{
  menubutton_ctl = new QToolButton();
  menubutton_ctl->setText("...");
  _layout->addWidget(menubutton_ctl);
  connect(menubutton_ctl, &QToolButton::clicked,
      this, &ThisClass::onMenuButtonClicked);

}

void QFFmpegOptionsControl::onMenuButtonClicked()
{
  QMenu menu;
  QAction * action = nullptr;

  menu.addAction(action = new QAction("List available encoders..."));
  connect(action, &QAction::triggered, [this]() {

      static const QString text = []() {

        QString text = "Tags:\n"
          "I - AV_CODEC_PROP_INTRA_ONLY, Codec uses only intra compression.\n"
          "L - AV_CODEC_PROP_LOSSY, Codec supports lossy compression. A codec may support both lossy and lossless compression modes\n"
          "S - AV_CODEC_PROP_LOSSLESS, Codec supports lossless compression. \n"
          "\n"
          "";

        for ( const auto & s : c_ffmpeg_writer::supported_encoders() ) {
          text.append(s.c_str());
          text.append("\n");
        }

        return text;
      }();

      QTextInfoDialogBox::show("Supported ffmpeg video encoders", text, this);
  });


  menu.addAction(action = new QAction("Supported pixel formats ..."));
  connect(action, &QAction::triggered, [this]() {

      static const QString text = []() {
        QString text = "Pixel formats:\n"
          "I.... = Supported Input  format for conversion\n"
          ".O... = Supported Output format for conversion\n"
          "..H.. = Hardware accelerated format\n"
          "...P. = Paletted format\n"
          "....B = Bitstream format\n"
          "FLAGS NAME            NB_COMPONENTS BITS_PER_PIXEL BIT_DEPTHS\n"
          "-----\n\n"
          "";

          for ( const auto & s : c_ffmpeg_writer::supported_pixel_formats() ) {
            text.append(s.c_str());
            text.append("\n");
          }
          return text;
      }();

      QTextInfoDialogBox::show("Supported pixel formats", text, this);
    });



  menu.exec(menubutton_ctl->mapToGlobal(
      QPoint(menubutton_ctl->width() / 2,
          menubutton_ctl->height() / 2)));

}

