/*
 * QImageScene.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageScene_h__
#define __QImageScene_h__

#include <QtWidgets/QtWidgets>

class QImageScene
    : public QGraphicsScene
{
  Q_OBJECT;
public:
  typedef QImageScene ThisClass;
  typedef QGraphicsScene Base;

  QImageScene(QObject * parent = Q_NULLPTR);
  QImageScene(const QRectF & sceneRect, QObject *parent  = Q_NULLPTR);
  QImageScene(qreal x, qreal y, qreal width, qreal height, QObject *parent  = Q_NULLPTR);

  QGraphicsPixmapItem * background(void) const;
  QGraphicsPixmapItem * setBackground(const QImage & image);
  QGraphicsPixmapItem * setBackground(const QPixmap & pxmap);

protected:
  void contextMenuEvent(QGraphicsSceneContextMenuEvent *e) override;

private:
  QGraphicsPixmapItem * bgItem_ = Q_NULLPTR;
};

#endif /* __QImageScene_h__ */
