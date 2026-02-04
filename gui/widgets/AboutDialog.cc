/*
 * AboutDialog.cc
 *
 *  Created on: Feb 4, 2026
 *      Author: amyznikov
 */

#include "AboutDialog.h"
#include <core/version.h>
#include <core/settings.h>
#include <opencv2/opencv.hpp>

#define TXTAUX1(s) #s
#define TXTAUX(s) TXTAUX1(s)
#define LIBCONFIG_VERSION_STR  TXTAUX(LIBCONFIG_VER_MAJOR) "." TXTAUX(LIBCONFIG_VER_MINOR) "." TXTAUX(LIBCONFIG_VER_REVISION)


//#LIBCONFIG_VER_MINOR##"."#LIBCONFIG_VER_REVISION


#define HAVE_FFMPEG ((HAVE_AVCODEC) && (HAVE_AVFORMAT) && (HAVE_AVUTIL) && (HAVE_SWSCALE))
#if HAVE_FFMPEG
extern "C" {
# include <libavcodec/avcodec.h>
# include <libavformat/avformat.h>
# include <libavutil/ffversion.h>
}
#endif

//#define APP_NAME    "SerStacker"
//#define APP_ICON    ":/serstacker/icons/app-icon.png"

AboutDialog::AboutDialog(const QString appName, const QPixmap & appIcon, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(tr("About Application"));
  setMinimumSize(450, 400);

  auto * mainLayout = new QVBoxLayout(this);

  auto * headerLayout = new QHBoxLayout();
  auto * iconLabel = new QLabel();
  if( !appIcon.isNull() ) {
    iconLabel->setPixmap(appIcon.scaled(64, 64, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }

  auto * titleLabel = new QLabel(QString("<b>%1</b><br>v%2").arg(appName).arg(PROJECT_FULL_VERSION_STR));
  titleLabel->setStyleSheet("font-size: 12pt;");

  headerLayout->addWidget(iconLabel);
  headerLayout->addSpacing(15);
  headerLayout->addWidget(titleLabel);
  headerLayout->addStretch();

  mainLayout->addLayout(headerLayout);
  mainLayout->addSpacing(10);

  QString info;
  info += QString("Full Version:\t\t%1\n").arg(PROJECT_FULL_VERSION_STR);
  info += QString("Git Commit:\t\t%1\n").arg(PROJECT_GIT_HASH);
  info += "------------------------------------------------------------\n";
  info += QString("Qt Runtime:\t\t%1\n").arg(qVersion());
  info += QString("Qt Compiled:\t%1\n").arg(QT_VERSION_STR);
  info += QString("OpenCV:\t\t%1\n").arg(CV_VERSION);
#if HAVE_FFMPEG
  info += QString("FFmpeg Codec:\t%1\n").arg(LIBAVCODEC_IDENT);
  info += QString("FFmpeg Format:\t%1\n").arg(LIBAVFORMAT_IDENT);
#endif
  info += QString("libconfig:\t\t%1\n").arg(LIBCONFIG_VERSION_STR);


  auto * infoText = new QTextEdit();
  infoText->setReadOnly(true);
  infoText->setPlainText(info);
  mainLayout->addWidget(infoText);

  auto * buttonLayout = new QHBoxLayout();
  auto * copyBtn = new QPushButton(tr("Copy to Clipboard"));
  auto * closeBtn = new QPushButton(tr("Close"));
  closeBtn->setDefault(true);

  buttonLayout->addWidget(copyBtn);
  buttonLayout->addStretch();
  buttonLayout->addWidget(closeBtn);
  mainLayout->addLayout(buttonLayout);

  connect(copyBtn, &QPushButton::clicked, [infoText, copyBtn]() {
    QApplication::clipboard()->setText(infoText->toPlainText());
    copyBtn->setText(tr("Copied!"));
  });

  connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
}

