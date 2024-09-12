/*
 * QGeoViewWidgetDock.h
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoViewWidgetDock_h__
#define __QGeoViewWidgetDock_h__

#include <gui/qcustomdock/QCustomDock.h>
#include "QGeoViewWidget.h"

class QGeoViewWidgetDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QGeoViewWidgetDock ThisClass;
  typedef QCustomDockWidget Base;

  QGeoViewWidgetDock(const QString &title, QWidget * parent = nullptr);

  QGeoViewWidget * geoView() const;

protected:
  QGeoViewWidget * _geoView = nullptr;
};


QGeoViewWidgetDock * addGeoViewWidgetDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu);

#endif /* __QGeoViewWidgetDock_h__ */
