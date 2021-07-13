/*
 * addctrl.cc
 *
 *  Created on: Jan 9, 2019
 *      Author: amyznikov
 */
#include "addctrl.h"


QDockWidget * addDock(QMainWindow * parent, Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QWidget * clientWidget,
    QMenu * viewMenu)
{
  QDockWidget * dock = new QDockWidget(title, parent);

  parent->addDockWidget(area, dock);

  dock->setAllowedAreas(Qt::AllDockWidgetAreas);
  dock->setObjectName(dockName);

  if ( clientWidget ) {
    dock->setWidget(clientWidget);
  }

  if ( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  QObject::connect(dock->toggleViewAction(), &QAction::triggered,
      [dock] (bool checked) {
        if ( checked ) {
          dock->raise();
        }
      });

  return dock;
}

