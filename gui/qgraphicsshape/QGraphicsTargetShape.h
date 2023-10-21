/*
 * QGraphicsTargetShape.h
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsTargetShape_h__
#define __QGraphicsTargetShape_h__

#include "QGraphicsShape.h"

class QGraphicsTargetShape:
    public QGraphicsShape
{
  Q_OBJECT;
public:
  typedef QGraphicsTargetShape ThisClass;
  typedef QGraphicsShape Base;

  explicit QGraphicsTargetShape(QGraphicsItem * parent = nullptr);

  void setCenter(const QPointF & p);
  const QPointF & center() const;

  void setFixOnSceneCenter(bool v);
  bool fixOnSceneCenter() const;

  void setLockPosition(bool v);
  bool lockPosition() const;

  void setBaseRadius(double v);
  double baseRadius() const;

  void setNumRings(int v);
  int numRings() const;

  void setShowDiagonalRays(bool v);
  bool showDiagonalRays() const;

  void setPen(const QPen & pen);
  void setCosmeticPen(const QColor & color, int width = 1 );
  const QPen & pen() const;

  void setPenWidth(int v);
  int penWidth() const;

  void setPenColor(const QColor & color);
  QColor penColor() const;

  void showShapeSettings();

protected:
  void updateGeometry();
  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  bool popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) override;
  void mousePressEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent * event) override;
  void onSceneChange() override;
  void onSceneHasChanged() override;
  void onSceneRectChanged(const QRectF &rect);



protected:
  QPointF center_;
  double baseRadius_ = 30;
  int numRings_ = 4;
  bool showDiagonals_ = false;
  bool fixOnSceneCenter_ = true;
  bool lockPosition_ = false;

  QRectF boundingRect_;
  QPainterPath shape_;
  QPen pen_;
};

#endif /* __QGraphicsTargetShape_h__ */
