/*
 * QGeoScene.cc
 *
 *  Created on: Nov 25, 2022
 *      Author: amyznikov
 */

#include "QGeoScene.h"

QGeoScene::QGeoScene(QObject *parent) :
  Base(parent)
{
}

QGeoProjection * QGeoScene::projection() const
{
  return projection_;
}

void QGeoScene::setProjection(QGeoProjection * projection)
{
  projection_ = projection;
  refreshProjection();
}

void QGeoScene::refreshProjection()
{
  Base::clear();

  if( projection_ ) {

    QRectF sceneRect =
        projection_->boundaryProjRect();

    const double viewXSize = 640;
    const double viewYSize = 480;

    const double projXSize = sceneRect.width();
    const double projYSize = sceneRect.height();

    const double newMinScaleFactor0 = 1.0;
    const double newMinScaleFactor1 = qAbs(viewXSize / projXSize);
    const double newMinScaleFactor2 = qAbs(viewYSize / projYSize);
    const double minScale = qMin(newMinScaleFactor0, qMin(newMinScaleFactor1, newMinScaleFactor2));
    const double maxScale = 16.0;

    Q_EMIT scaleLimitsChanged(minScale, maxScale);

    const double offset = 1;

    sceneRect.adjust(-sceneRect.width() * offset,
        -sceneRect.height() * offset,
        +sceneRect.width() * offset,
        +sceneRect.height() * offset);

    setSceneRect(sceneRect);
  }
}

bool QGeoScene::popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  return false;
}

void QGeoScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *event)
{
  event->ignore();

  Base::contextMenuEvent(event);
  if ( event->isAccepted() ) {
    return;
  }

  Q_EMIT contextMenuRequested(event);
}
