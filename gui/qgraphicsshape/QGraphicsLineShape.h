/*
 * QGraphicsLineShape.h
 *
 *  Created on: Jan 16, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsLineShape_h__
#define __QGraphicsLineShape_h__

#include "QGraphicsShape.h"

class QGraphicsLineShape:
    public QGraphicsShape
{
  Q_OBJECT;
public:
  typedef QGraphicsLineShape ThisClass;
  typedef QGraphicsShape Base;

  explicit QGraphicsLineShape(QGraphicsItem * parent = nullptr);
  explicit QGraphicsLineShape(const QLineF & line, QGraphicsItem * parent = nullptr);
  explicit QGraphicsLineShape(const QPointF & p1, const QPointF & p2,  QGraphicsItem * parent = nullptr);
  explicit QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem * parent = nullptr);

  void setLine(const QLineF & line);
  void setLine(qreal x1, qreal y1, qreal x2, qreal y2);
  const QLineF& line() const;

  void setSceneLine(const QLineF & line);
  void setSceneLine(qreal x1, qreal y1, qreal x2, qreal y2);
  QLineF sceneLine() const;

  void setPen(const QPen & pen);
  void setCosmeticPen(const QColor & color, int width = 1);
  const QPen& pen() const;

  void setPenWidth(int v);
  int penWidth() const;

  void setPenColor(const QColor & color);
  QColor penColor() const;

  void setLockP1( bool v);
  bool lockP1() const;

  void setLockP2( bool v);
  bool lockP2() const;

protected:
  void updateGeometry();
  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  void mousePressEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent * event) override;
  bool popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) override;

protected:
  QLineF line_;
  QRectF boundingRect_;
  QPainterPath shape_;
  QPen pen_;
  QAction * lockP1Action_ = nullptr;
  QAction * lockP2Action_ = nullptr;

  enum MouseAction {
    MouseAction_None = 0,
    MouseAction_MoveP1,
    MouseAction_MoveP2,
    MouseAction_MoveWholeLine,
  } currentMouseAction_ = MouseAction_None;

  bool lockP1_ = false;
  bool lockP2_ = false;
};

#endif /* __QGraphicsLineShape_h__ */
