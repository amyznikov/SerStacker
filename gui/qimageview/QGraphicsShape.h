/*
 * QGraphicsShape.h
 *
 *  Created on: Oct 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsShape_h__
#define __QGraphicsShape_h__

#include <QtWidgets/QtWidgets>

class QGraphicsShape :
    public QObject
{
public:
  virtual void populateContextMenu(QMenu & menu, const QGraphicsSceneContextMenuEvent & e)
  {
  }
};

class QGraphicsLineShape :
    public QGraphicsShape,
    public QGraphicsLineItem
{
  Q_OBJECT;
public:
  typedef QGraphicsLineShape ThisClass;
  typedef QGraphicsLineItem Base;

  explicit QGraphicsLineShape(QGraphicsItem *parent = nullptr);
  explicit QGraphicsLineShape(const QLineF &line, QGraphicsItem *parent = nullptr);
  explicit QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem *parent = nullptr);

  QPainterPath shape() const override;
  QRectF boundingRect() const override;
  void populateContextMenu(QMenu & menu, const QGraphicsSceneContextMenuEvent & e) override;

signals:
  void onItemChanged(QGraphicsLineItem*);

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

protected:
  enum line_moving_mode {
    not_moving = 0,
    move_p1,
    move_p2,
    move_whole_line,
  } line_moving_mode_ = not_moving;

  bool p1Locked_ = false;
  bool p2Locked_ = false;
};

class QGraphicsRectShape :
    public QGraphicsShape,
    public QGraphicsRectItem
{
  Q_OBJECT;
public:
  typedef QGraphicsRectShape ThisClass;
  typedef QGraphicsRectItem Base;

  explicit QGraphicsRectShape(QGraphicsItem *parent = nullptr);
  explicit QGraphicsRectShape(const QRectF &rect, QGraphicsItem *parent = nullptr);
  explicit QGraphicsRectShape(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent = nullptr);

  QPainterPath shape() const override;
  QRectF boundingRect() const override;

signals:
  void onItemChanged(QGraphicsRectItem*);

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

protected:
  enum rect_moving_mode {
    not_moving = 0,
    move_lt,
    move_rt,
    move_rb,
    move_lb,
    move_whole_rect,
  } rect_moving_mode_ = not_moving;
};


#endif /* __QGraphicsShape_h__ */
