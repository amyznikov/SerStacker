/*
 * QFFmpegOptionsControl.cc
 *
 *  Created on: Feb 18, 2024
 *      Author: amyznikov
 */

#include "QFFmpegOptionsControl.h"
#include <gui/widgets/QLineEditBox.h>
#include <gui/widgets/createAction.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/debug.h>

namespace {

class QPlainTextDialogBox :
    public QDialog
{
public:
  typedef QPlainTextDialogBox ThisClass;
  typedef QDialog Base;

  QPlainTextDialogBox(QWidget * parent = nullptr);

  void setText(const QString & text);

protected:
  QToolBar * toolbar_ctl = nullptr;
  QPlainTextEdit * textbox_ctl = nullptr;
  QLineEditBox * searchText_ctl = nullptr;
  QAction * searchNext_ctl = nullptr;
};

QPlainTextDialogBox::QPlainTextDialogBox(QWidget * parent) :
    Base(parent)
{
  QVBoxLayout * vbox =
      new QVBoxLayout(this);

  vbox->addWidget(toolbar_ctl =
      new QToolBar(this));

  vbox->addWidget(textbox_ctl =
      new QPlainTextEdit(this));

  toolbar_ctl->addWidget(searchText_ctl =
      new QLineEditBox(this));

  textbox_ctl->setReadOnly(true);
  textbox_ctl->setFont(QFont("Monospace", 14));
  textbox_ctl->setWordWrapMode(QTextOption::WrapMode::NoWrap);

  connect(searchText_ctl, &QLineEditBox::textChanged,
      [this]() {
        searchNext_ctl->setEnabled(!searchText_ctl->text().isEmpty());
      });

  toolbar_ctl->addAction(searchNext_ctl =
      createAction(QIcon(), "Search Next",
          "Search next occurrence of text",
          [this]() {
            const QString text = searchText_ctl->text();
            if ( !text.isEmpty() && !textbox_ctl->find(text) ) {
            }
          }));

  searchNext_ctl->setEnabled(!searchText_ctl->text().isEmpty());

  connect(searchText_ctl, &QLineEditBox::returnPressed,
      searchNext_ctl, &QAction::trigger);
}

void QPlainTextDialogBox::setText(const QString & text)
{
  textbox_ctl->setPlainText(text);
}

} // namespace


QFFmpegOptionsControl::QFFmpegOptionsControl(QWidget * parent) :
  Base(parent)
{
  menubutton_ctl = new QToolButton();
  menubutton_ctl->setText("...");
  layout_->addWidget(menubutton_ctl);
  connect(menubutton_ctl, &QToolButton::clicked,
      this, &ThisClass::onMenuButtonClicked);

}

void QFFmpegOptionsControl::onMenuButtonClicked()
{
  QMenu menu;
  QAction * action = nullptr;

  menu.addAction(action = new QAction("List available encoders..."));
  connect(action, &QAction::triggered,
      []() {

        static QPlainTextDialogBox * codecListTextBox = nullptr;

        if ( !codecListTextBox ) {

          const std::vector<std::string> & supported_encoders =
              c_ffmpeg_writer::supported_encoders();

          QString text = "Tags:\n"
              "I - AV_CODEC_PROP_INTRA_ONLY, Codec uses only intra compression.\n"
              "L - AV_CODEC_PROP_LOSSY, Codec supports lossy compression. A codec may support both lossy and lossless compression modes\n"
              "S - AV_CODEC_PROP_LOSSLESS, Codec supports lossless compression. \n"
              "\n"
              "";

          for ( const auto & s : supported_encoders ) {
            text.append(s.c_str());
            text.append("\n");
          }

          codecListTextBox = new QPlainTextDialogBox(QApplication::activeWindow());
          codecListTextBox->setText(text);
          codecListTextBox->setWindowTitle("Supported ffmpeg video encoders");
        }


        codecListTextBox->show();
        codecListTextBox->raise();
        codecListTextBox->setFocus();
      });


  menu.addAction(action = new QAction("Supported pixel formats ..."));
  connect(action, &QAction::triggered,
      []() {

        static QPlainTextDialogBox * codecListTextBox = nullptr;

        if ( !codecListTextBox ) {

          const std::vector<std::string> & supported_pixel_formats =
              c_ffmpeg_writer::supported_pixel_formats();

          QString text = "Pixel formats:\n"
              "I.... = Supported Input  format for conversion\n"
              ".O... = Supported Output format for conversion\n"
              "..H.. = Hardware accelerated format\n"
              "...P. = Paletted format\n"
              "....B = Bitstream format\n"
              "FLAGS NAME            NB_COMPONENTS BITS_PER_PIXEL BIT_DEPTHS\n"
              "-----\n\n"
              "";

          for ( const auto & s : supported_pixel_formats ) {
            text.append(s.c_str());
            text.append("\n");
          }

          codecListTextBox = new QPlainTextDialogBox(QApplication::activeWindow());
          codecListTextBox->setText(text);
          codecListTextBox->setWindowTitle("Supported pixel formats");
        }

        codecListTextBox->show();
        codecListTextBox->raise();
        codecListTextBox->setFocus();
      });



  menu.exec(menubutton_ctl->mapToGlobal(
      QPoint(menubutton_ctl->width() / 2,
          menubutton_ctl->height() / 2)));

}
