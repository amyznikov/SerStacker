/*
 * main.cc
 *
 *  Created on September 5, 2019
 *      Author: amyznikov
 */
#include <opencv2/core/ocl.hpp>
#include <gui/widgets/style.h>
#include "MainWindow.h"
#include <core/readdir.h>
#include <core/debug.h>


#define MY_COMPANY  "amyznikov"
#define MY_APP      "SerStacker"

using namespace serstacker;

int main(int argc, char *argv[])
{
  // My OpenCV build has problems with OCL.
  // Comment out this line if you want to allow OCL in OpenCV
  cv::ocl::setUseOpenCL(false);

  QApplication app(argc, argv);

  app.setOrganizationName(MY_COMPANY);
  app.setApplicationName(MY_APP);

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);

  if ( true ) {

    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::defaultFormat(), QSettings::UserScope,
        QString("%1/.config").arg(get_home_directory().c_str()));
  }


  Q_INIT_RESOURCE(app_resources);
  Q_INIT_RESOURCE(gui_resources);


  setIconStyleSelector("dark");


  MainWindow mainWindow;
  mainWindow.show();
  mainWindow.activateWindow();
  mainWindow.raise();


  return app.exec();
}
