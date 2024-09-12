/*
 * QGeoViewWidgetDock.cc
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#include "QGeoViewWidgetDock.h"

QGeoViewWidgetDock::QGeoViewWidgetDock(const QString &title, QWidget * parent) :
  Base(title, parent)
{
  Base::setWidget(_geoView = new QGeoViewWidget(this));

  const QList<QAction*> actions = _geoView->toolbarActions();
  for( int i = 0, n = actions.size(); i < n; ++i ) {
    titleBar()->addButton(actions[n - i - 1]);
  }

}


QGeoViewWidget * QGeoViewWidgetDock::geoView() const
{
  return _geoView;
}


QGeoViewWidgetDock * addGeoViewWidgetDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QGeoViewWidgetDock *dock = new QGeoViewWidgetDock(title, parent);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  return dock;
}
