/*
 * QGeoScene.h
 *
 *  Created on: Nov 25, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoScene_h__
#define __QGeoScene_h__

#include <QtWidgets/QtWidgets>

#include "QGeoProjection.h"

class QGeoScene:
    public QGraphicsScene
{
  Q_OBJECT;
public:
  typedef QGeoScene ThisClass;
  typedef QGraphicsScene Base;

  QGeoScene(QObject *parent = nullptr);

  QGeoProjection * projection() const;
  void setProjection(QGeoProjection * projection);
  void refreshProjection();

  virtual bool popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu);

Q_SIGNALS:
  void scaleLimitsChanged(double minScale, double maxScale);
  void contextMenuRequested(QGraphicsSceneContextMenuEvent *event);

protected:
  void contextMenuEvent(QGraphicsSceneContextMenuEvent *event) override;

protected:
  QGeoProjection * projection_  = nullptr;
};

#endif /* __QGeoScene_h__ */
