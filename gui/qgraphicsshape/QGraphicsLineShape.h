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

  enum AlignMode {
    AlignNone,
    AlignVert,
    AlignHorz,
  };

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

  void setArrowSize(double v);
  double arrowSize() const;

  void alignVertically();
  void alignHorizontally();

  void setAlignMode(AlignMode v);
  AlignMode alignMode() const;

  void popuateContextMenu(QMenu & menu, const QPoint & viewpos) override;

protected:
  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  void mousePressEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent * event) override;
  void updateGeometry() override;
  void showShapeSettings();

protected:
  QLineF _line;
  double _arrowSize = 5;
  QRectF _boundingRect;
  QPainterPath _shape;
  QPen _pen;
  QPointF _lastPos;

  enum MouseAction {
    MouseAction_None = 0,
    MouseAction_MoveP1,
    MouseAction_MoveP2,
    MouseAction_MoveWholeLine,
  } currentMouseAction_ = MouseAction_None;

  AlignMode _alignMode = AlignNone;
  bool _lockP1 = false;
  bool _lockP2 = false;
};

#endif /* __QGraphicsLineShape_h__ */
