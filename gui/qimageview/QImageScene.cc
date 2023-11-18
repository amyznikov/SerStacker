/*
 * QImageScene.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageScene.h"

QImageScene::QImageScene(QObject * parent)
  : Base(parent)
{
  pixmapItem_ = addPixmap(QPixmap());
  pixmapItem_->setZValue(-1000);
  pixmapItem_->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  pixmapItem_->setFlags(pixmapItem_->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  pixmapItem_->setPos(pixmapItem_->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QImageScene::QImageScene(const QRectF & sceneRect, QObject *parent)
  : Base(sceneRect, parent)
{
  pixmapItem_ = addPixmap(QPixmap());
  pixmapItem_->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  pixmapItem_->setFlags(pixmapItem_->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  pixmapItem_->setPos(pixmapItem_->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QImageScene::QImageScene(qreal x, qreal y, qreal width, qreal height, QObject *parent)
  : Base(x, y, width, height, parent)
{
  pixmapItem_ = addPixmap(QPixmap());
  pixmapItem_->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  pixmapItem_->setFlags(pixmapItem_->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  pixmapItem_->setPos(pixmapItem_->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QGraphicsPixmapItem * QImageScene::image(void) const
{
  return pixmapItem_;
}

QGraphicsPixmapItem * QImageScene::setImage(const QImage & image)
{
  return setImage(QPixmap::fromImage(image));
}

QGraphicsPixmapItem * QImageScene::setImage(const QPixmap & pxmap)
{
  const QSize oldPixmapSize =
      pixmapItem_->pixmap().size();

  pixmapItem_->setPixmap(pxmap);
  pixmapItem_->setVisible(true);

  if ( !pxmap.isNull() && pxmap.size() != oldPixmapSize ) {
    setSceneRect(0, 0, pxmap.width(), pxmap.height());
    pixmapItem_->setPos(0, 0);
  }

  return pixmapItem_;
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

