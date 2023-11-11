/*
 * QImageScene.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageScene.h"
//#include "QGraphicsShape.h"
#include <core/debug.h>

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

  //  QRectF rc = sceneRect();
  //  CF_DEBUG("oldPixmapSize: %dx%d", oldPixmapSize.width(), oldPixmapSize.height());
  //  CF_DEBUG("pxmap.size(): %dx%d", pxmap.width(), pxmap.height());
  //  CF_DEBUG("sceneRect: %g %g %gx%g", rc.x(), rc.y(), rc.width(), rc.height());

  return pixmapItem_;
}

void QImageScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *e)
{
  //  menu.addAction("Send to back",
  //      [this]() {
  //
  //      //this->stackBefore();
  //
  //        CF_DEBUG("zValue=%g", this->zValue());
  //
  //        QGraphicsScene * scene =
  //            this->scene();
  //        if ( scene ) {
  //          QList<QGraphicsItem *> collidingItems = scene->collidingItems(this, Qt::IntersectsItemShape);
  //          CF_DEBUG("collidingItems.size=%d", collidingItems.size());
  //          if ( collidingItems.size() > 1 ) {
  //            //scene->
  //
  //          }
  //
  //        }
  //
  //  });
  //
  //  menu.addSeparator();

  Base::contextMenuEvent(e);
}

void QImageScene::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
//  CF_DEBUG("QImageScene: ENTER e->isAccepted()=%d", e->isAccepted());
  //e->ignore();
  Base::mousePressEvent(e);
//  CF_DEBUG("QImageScene: LEAVE e->isAccepted()=%d", e->isAccepted());

}

void QImageScene::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
{
  if( e->buttons() != Qt::NoButton ) {
    //e->ignore();
    Base::mouseMoveEvent(e);
    //CF_DEBUG("QImageScene: e->isAccepted()=%d", e->isAccepted());
  }
}

void QImageScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
  //e->ignore();
  Base::mouseReleaseEvent(e);
}

