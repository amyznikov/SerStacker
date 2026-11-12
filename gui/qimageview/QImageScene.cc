/*
 * QImageScene.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageScene.h"
#include <core/debug.h>

QImageScene::QImageScene(QObject * parent) :
  Base(parent)
{
  _pxmapItem = addPixmap(QPixmap());
  _pxmapItem->setZValue(-1000);
  _pxmapItem->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  _pxmapItem->setFlags(_pxmapItem->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  _pxmapItem->setPos(_pxmapItem->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QImageScene::QImageScene(const QRectF & sceneRect, QObject *parent) :
    Base(sceneRect, parent)
{
  _pxmapItem = addPixmap(QPixmap());
  _pxmapItem->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  _pxmapItem->setFlags(_pxmapItem->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  _pxmapItem->setPos(_pxmapItem->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QImageScene::QImageScene(qreal x, qreal y, qreal width, qreal height, QObject *parent) :
    Base(x, y, width, height, parent)
{
  _pxmapItem = addPixmap(QPixmap());
  _pxmapItem->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  _pxmapItem->setFlags(_pxmapItem->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  _pxmapItem->setPos(_pxmapItem->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QGraphicsPixmapItem * QImageScene::image(void) const
{
  return _pxmapItem;
}

QGraphicsPixmapItem * QImageScene::setImage(const QImage & image)
{
  return setImage(QPixmap::fromImage(image));
}

QGraphicsPixmapItem * QImageScene::setImage(const QPixmap & pxmap)
{
  const QSize oldPixmapSize = _pxmapItem->pixmap().size();
  _pxmapItem->setPixmap(pxmap);
  _pxmapItem->setVisible(true);

  if ( !pxmap.isNull() && pxmap.size() != oldPixmapSize ) {
    setSceneRect(0, 0, pxmap.width(), pxmap.height());
    _pxmapItem->setPos(0, 0);
  }

  return _pxmapItem;
}

void QImageScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *e)
{
  Base::contextMenuEvent(e);
}

void QImageScene::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
  Base::mousePressEvent(e);
}

void QImageScene::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
{
  if( e->buttons() != Qt::NoButton ) {
    Base::mouseMoveEvent(e);
  }
}

void QImageScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
  //e->ignore();
  Base::mouseReleaseEvent(e);
}

