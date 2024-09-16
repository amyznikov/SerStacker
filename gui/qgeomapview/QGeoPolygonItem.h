/*
 * QGeoPolygonItem.h
 *
 *  Created on: Nov 27, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoMapPolygonItem_h__
#define __QGeoMapPolygonItem_h__

#include "QGeoGraphicsItem.h"

class QAbstractGeoPolygonItem :
    public QGeoGraphicsItem
{
  Q_OBJECT;
public:
  typedef QAbstractGeoPolygonItem ThisClass;
  typedef QGeoGraphicsItem Base;

  QAbstractGeoPolygonItem(QGraphicsItem *parent = nullptr);
  QAbstractGeoPolygonItem(const QString & name, const QString & description, QGraphicsItem *parent = nullptr);
  ~QAbstractGeoPolygonItem();
  Q_DISABLE_COPY(QAbstractGeoPolygonItem);

  void setFillRule(Qt::FillRule v);
  Qt::FillRule fillRule() const;

  void setEnableMovePoints(bool v);
  bool enableMovePoints() const;

  void setEnableAddPoints(bool v);
  bool enableAddPoints() const;

  void setEnableRemovePoints(bool v);
  bool enableRemovePoints() const;

  bool populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu) override;

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
  QPen pen_;
  QBrush brush_;
  QPolygonF projPoints_;
  QPainterPath projShape_;
  QRectF boundingRect_;
  Qt::FillRule fillRule_ = Qt::OddEvenFill;
  int currentMovingPointIndex_ = -1;
  bool enableMovePoints_ = true;
  bool enableAddPoints_ = true;
  bool enableRemovePoints_ = true;
  bool contextMenuRequest_ = false;
};


class QGeoPolygonItem:
    public QAbstractGeoPolygonItem
{
public:
  typedef QGeoPolygonItem ThisClass;
  typedef QAbstractGeoPolygonItem Base;

  QGeoPolygonItem(QGraphicsItem *parent = nullptr);
  QGeoPolygonItem(const QString & name, const QString & description, QGraphicsItem *parent = nullptr);

  void setPoints(const QVector<QGeoPos> & points);
  void setPoints(const std::vector<QGeoPos> & points);
  void setPoints(const QGeoPos points[], int count);

protected: // QAbstractGeoPolygonItem
  int pointsCount() const override;
  void insertPoint(const QGeoPos &, int insert_pos) override;
  void removePoint(int remove_pos) override;
  void setGeoPoint(int index, const QGeoPos & ) override;
  QGeoPos getGeoPoint(int index) const override;

protected:
  std::vector<QGeoPos> geoPoints_;
};


#endif /* __QGeoMapPolygonItem_h__ */
