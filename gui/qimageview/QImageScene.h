/*
 * QImageScene.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageScene_h__
#define __QImageScene_h__

#include <QtWidgets/QtWidgets>

class QImageScene :
    public QGraphicsScene
{
  Q_OBJECT;
public:
  typedef QImageScene ThisClass;
  typedef QGraphicsScene Base;

  QImageScene(QObject * parent = nullptr);
  QImageScene(const QRectF & sceneRect, QObject *parent  = nullptr);
  QImageScene(qreal x, qreal y, qreal width, qreal height, QObject *parent  = nullptr);

  QGraphicsPixmapItem * image(void) const;
  QGraphicsPixmapItem * setImage(const QImage & image);
  QGraphicsPixmapItem * setImage(const QPixmap & pxmap);

Q_SIGNALS:
  void graphicsItemChanged(QGraphicsItem * item);
  void graphicsItemVisibleChanged(QGraphicsItem * item);
  void graphicsItemDestroyed(QGraphicsItem * item);

protected:
  void contextMenuEvent(QGraphicsSceneContextMenuEvent *e) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

private:
  QGraphicsPixmapItem * pixmapItem_ = nullptr;
};

#endif /* __QImageScene_h__ */
