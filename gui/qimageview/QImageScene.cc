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
  bgItem_ = addPixmap(QPixmap());
  bgItem_->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  bgItem_->setFlags(bgItem_->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  bgItem_->setPos(bgItem_->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QImageScene::QImageScene(const QRectF & sceneRect, QObject *parent)
  : Base(sceneRect, parent)
{
  bgItem_ = addPixmap(QPixmap());
  bgItem_->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  bgItem_->setFlags(bgItem_->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  bgItem_->setPos(bgItem_->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QImageScene::QImageScene(qreal x, qreal y, qreal width, qreal height, QObject *parent)
  : Base(x, y, width, height, parent)
{
  bgItem_ = addPixmap(QPixmap());
  bgItem_->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
  bgItem_->setFlags(bgItem_->flags() &
      ~(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemIsFocusable));
  bgItem_->setPos(bgItem_->mapFromScene(0, 0));

  setSceneRect(0, 0, 1, 1);
}

QGraphicsPixmapItem * QImageScene::background(void) const
{
  return bgItem_;
}

QGraphicsPixmapItem * QImageScene::setBackground(const QImage & image)
{
  return setBackground(QPixmap::fromImage(image));
}

QGraphicsPixmapItem * QImageScene::setBackground(const QPixmap & pxmap)
{
  bgItem_->setPixmap(pxmap);
  if ( !pxmap.isNull() ) {
    setSceneRect(0, 0, pxmap.width(), pxmap.height());
    bgItem_->setPos(bgItem_->mapFromScene(0, 0));
  }
  return bgItem_;
}

void QImageScene::contextMenuEvent(QGraphicsSceneContextMenuEvent */*e*/)
{
//  QGraphicsItem * item = itemAt(e->scenePos(), QTransform());
//  if (item ) {
//  }
}
