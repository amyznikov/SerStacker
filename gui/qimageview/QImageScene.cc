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

QGraphicsPixmapItem * QImageScene::sceneImage(void) const
{
  return pixmapItem_;
}

QGraphicsPixmapItem * QImageScene::setSceneImage(const QImage & image)
{
  return setSceneImage(QPixmap::fromImage(image));
}

QGraphicsPixmapItem * QImageScene::setSceneImage(const QPixmap & pxmap)
{
  const QSize oldPixmapSize =
      pixmapItem_->pixmap().size();

//  CF_DEBUG("oldPixmapSize: %dx%d", oldPixmapSize.width(), oldPixmapSize.height());
//  CF_DEBUG("pxmap.size(): %dx%d", pxmap.width(), pxmap.height());

  pixmapItem_->setPixmap(pxmap);
  pixmapItem_->setVisible(true);

  if ( !pxmap.isNull() && pxmap.size() != oldPixmapSize ) {
    setSceneRect(0, 0, pxmap.width(), pxmap.height());
    //pixmapItem_->setPos(pixmapItem_->mapFromScene(0, 0));
    pixmapItem_->setPos(0, 0);
  }

//  QRectF rc = sceneRect();
//  CF_DEBUG("sceneRect: %g %g %gx%g", rc.x(), rc.y(), rc.width(), rc.height());

  return pixmapItem_;
}

void QImageScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *e)
{
  Base::contextMenuEvent(e);

//  QGraphicsItem * item =
//      itemAt(e->scenePos(),
//          QTransform());
//
//  if ( !item || item == pixmapItem_ ) {
//    Base::contextMenuEvent(e);
//  }
//  else { // show context menu
//
//    QMenu menu;
//    QAction * action;
//
//    QGraphicsShape * shape =
//        dynamic_cast<QGraphicsShape * >(item);
//
//    if ( shape ) {
//      shape->populateContextMenu(menu, *e);
//      menu.addSeparator();
//    }
//
//    menu.addAction(action = new QAction("Delete this object"));
//    connect(action, &QAction::triggered,
//        [this, item]() {
//      removeItem(item);
//    });
//
//    menu.exec(e->screenPos());
//
//    e->accept();
//  }
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

