/*
 * main.cc
 *
 *  Created on September 5, 2019
 *      Author: amyznikov
 */
#include <opencv2/core/ocl.hpp>
#include <gui/widgets/style.h>
#include "MainWindow.h"
#include "camera/ffmpeg/QFFStreams.h"
#include "camera/libcamera-sctp/QLCSCTPStreams.h"
#include <core/readdir.h>
#include <core/debug.h>

#define MY_COMPANY  "amyznikov"
#define MY_APP      "SerImager"

using namespace serimager;

int main(int argc, char * argv[])
{
  // My OpenCV build has problems with OCL.
  // Comment out this line if you want to allow OCL in OpenCV
  // cv::ocl::setUseOpenCL(false);

  QApplication app(argc, argv);
  app.setOrganizationName(MY_COMPANY);
  app.setApplicationName(MY_APP);

  QFFStreams::registerMetaTypes();
  QLCSCTPStreams::registerMetaTypes();

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  if ( true ) {

    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::defaultFormat(), QSettings::UserScope,
        QString("%1/.config").arg(get_home_directory().c_str()));
  }

  Q_INIT_RESOURCE(gui_resources);
  Q_INIT_RESOURCE(app_resources);
  Q_INIT_RESOURCE(qdarkstyle);



  // speed-up using multithreads
  cv::setUseOptimized(true);
  cv::setNumThreads(4);


  bool useDarkStyle = true;
  if ( !useDarkStyle ) {
    setIconStyleSelector("dark");
  }
  else {

    setIconStyleSelector("light");

    //  QFont font("SansSerif", 16, QFont::Medium);
    //  font.setStyleHint(QFont::SansSerif);
    //  QFont font("System", 16, QFont::Medium);
    //  font.setStyleHint(QFont::System);
    //  app.setFont(font);

    QVersionNumber running_version =
        QVersionNumber::fromString(QString(qVersion()));

    QVersionNumber threshod_version(5, 13, 0);
    QString qss_resource(":/gui/qdarkstyle/style.qss");

    if( running_version >= threshod_version ) {
      qss_resource = ":/gui/qdarkstyle/style-5.13.qss";
    }

    QFile f(qss_resource);
    f.open(QFile::ReadOnly | QFile::Text);

    QTextStream ts(&f);
    app.setStyleSheet(ts.readAll());

    f.close();
  }

  // CF_DEBUG("PLATFORM=%s", QApplication::platformName().toUtf8().data());

  MainWindow mainWindow;

  mainWindow.show();
  mainWindow.activateWindow();
  mainWindow.raise();

  return app.exec();
}
