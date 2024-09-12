/*
 * QGeoPolyLineItem.h
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoPolyLineItem_h__
#define __QGeoPolyLineItem_h__

#include "QGeoGraphicsItem.h"


class QAbstractGeoPolyLineItem :
    public QGeoGraphicsItem
{
  Q_OBJECT;
public:
  typedef QAbstractGeoPolyLineItem ThisClass;
  typedef QGeoGraphicsItem Base;

  QAbstractGeoPolyLineItem(QGraphicsItem *parent = nullptr);
  QAbstractGeoPolyLineItem(const QString & name, const QString & description, QGraphicsItem *parent = nullptr);
  ~QAbstractGeoPolyLineItem();
  Q_DISABLE_COPY(QAbstractGeoPolyLineItem);

  void setFillRule(Qt::FillRule v);
  Qt::FillRule fillRule() const;

  void setEnableMovePoints(bool v);
  bool enableMovePoints() const;

  void setEnableAddPoints(bool v);
  bool enableAddPoints() const;

  void setEnableRemovePoints(bool v);
  bool enableRemovePoints() const;

  void setShowPoints(bool v);
  bool showPoints() const;

  void setShowLines(bool v);
  bool showLines() const;

  void setLineWidth(int v);
  int lineWidth() const;

  void setLineColor(const QColor & v);
  QColor lineColor() const;

  void setLineOpaqueness(int v);
  int lineOpaqueness() const;

  void setPointSize(int v);
  int pointSize() const;

  void setPointPenWidth(int v);
  int pointPenWidth() const;

  void setPointColor(const QColor & v);
  QColor pointColor() const;

  void setPointOpaqueness(int v);
  int pointOpaqueness() const;

  bool popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) override;

Q_SIGNALS:
  void geoPointsChanged();

protected:
  void updateProjected(const QGeoScene* scene =  nullptr) override;
  void updateGeo(const QGeoScene* scene = nullptr) override;

protected:
  virtual int pointsCount() const = 0;
  virtual void insertPoint(const QGeoPos &, int insert_pos) = 0;
  virtual void removePoint(int remove_pos) = 0;
  virtual void setGeoPoint(int index, const QGeoPos & ) = 0;
  virtual QGeoPos getGeoPoint(int index) const = 0;

protected: // QGraphicsObject
  QRectF boundingRect() const override;
  QPainterPath shape() const override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  void mousePressEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent * event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent * event) override;

protected:

protected:
  QPen linePen_;
  QPen pointPen_;
//  QBrush brush_;
  QPolygonF projPoints_;
  QPainterPath projShape_;
  QRectF boundingRect_;
  Qt::FillRule fillRule_ = Qt::OddEvenFill;
  int currentMovingPointIndex_ = -1;
  int pointSize_ = 1;
  bool enableMovePoints_ = true;
  bool enableAddPoints_ = true;
  bool enableRemovePoints_ = true;
  bool contextMenuRequest_ = false;
  bool showPoints_ = true;
  bool showLines_ = true;
};


class QGeoPolyLineItem :
    public QAbstractGeoPolyLineItem
{
public:
  QGeoPolyLineItem();
};

#endif /* __QGeoPolyLineItem_h__ */
