/*
 * QGraphicsRectShape.h
 *
 *  Created on: Jan 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsRectShape_h__
#define __QGraphicsRectShape_h__

#include "QGraphicsShape.h"

class QGraphicsRectShape :
    public QGraphicsShape
{
  Q_OBJECT;
public:
  typedef QGraphicsRectShape ThisClass;
  typedef QGraphicsShape Base;

  enum MouseAction {
    MouseAction_None,
    MouseAction_MoveRect,
    MouseAction_MoveTop,
    MouseAction_MoveRight,
    MouseAction_MoveBottom,
    MouseAction_MoveLeft,
    MouseAction_MoveTopLeft,
    MouseAction_MoveTopRight,
    MouseAction_MoveBottomRight,
    MouseAction_MoveBottomLeft,
  };


  QGraphicsRectShape(QGraphicsItem * parent = nullptr);
  QGraphicsRectShape(const QRectF & rect, QGraphicsItem * parent = nullptr);
  QGraphicsRectShape(const QString & name, const QString & description, QGraphicsItem *parent = nullptr);
  QGraphicsRectShape(const QString & name, const QString & description, const QRectF & rect, QGraphicsItem *parent = nullptr);

  void setResizable(bool v);
  bool resizable() const;

  void setRect(const QRectF & rc);
  void setSceneRect(const QPointF & topLeft, const QPointF & bottomRight);
  const QRectF & rect() const;
  QRectF sceneRect() const;
  QRect iSceneRect() const;

  void setCenter(const QPointF & p);
  QPointF center() const;

  void setFixOnSceneCenter(bool v);
  bool fixOnSceneCenter() const;

  void setPen(const QPen & pen);
  void setCosmeticPen(const QColor & color, int width = 1 );
  const QPen & pen() const;

  void setBrush(const QBrush & brush);
  const QBrush & brush() const;

  void setPenWidth(int v);
  int penWidth() const;

  void setPenColor(const QColor & color);
  QColor penColor() const;

  void showShapeSettings();

  void popuateContextMenu(QMenu & menu, const QPoint & viewpos) override;

protected:
  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  void mousePressEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent * event) override;
  void onSceneChange() override;
  void onSceneHasChanged() override;
  void onSceneRectChanged(const QRectF &rect);
  void updateGeometry() override;

protected:
  QRectF _rect;
  QRectF _boundingRect;
  QPainterPath _shape;
  QPen _pen;
  QBrush _brush;
  int _hitDstance = 15;
  MouseAction _currentMouseAction = MouseAction_None;
  QPointF _mdelta;
  bool _itemIsResizable = true;
  bool _fixOnSceneCenter = false;
};

#endif /* __QGraphicsRectShape_h__ */
