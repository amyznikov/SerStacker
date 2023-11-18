/*
 * QGeoTiles.h
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoTiles_h__
#define __QGeoTiles_h__

#include "QGeoGraphicsItem.h"
#include "QGeoTilePos.h"


class QGeoTiles :
    public QGeoGraphicsItem
{
public:
  typedef QGeoTiles ThisClass;
  typedef QGeoGraphicsItem Base;

  QGeoTiles(QGraphicsItem *parent = nullptr);
  QGeoTiles(const QString & name, const QString & description, QGraphicsItem *parent = nullptr);
  Q_DISABLE_COPY(QGeoTiles);

  void onCamera(const QGeoViewCameraState& oldState, const QGeoViewCameraState & newState) override;
  void processCamera(const QGeoViewCameraState & newState, bool force = false);

  virtual int minZoomlevel() const = 0;
  virtual int maxZoomlevel() const = 0;
  virtual int scaleToZoom(double scale) const;
  virtual void request(const QGeoTilePos& tilePos) = 0;
  virtual void cancel(const QGeoTilePos& tilePos) = 0;


protected: // QGraphicsObject
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

protected: // QGeoMapGraphicsItem
  void onTile(const QGeoTilePos& tilePos, QGraphicsItem * tileObj);

protected:
  void removeAllAbove(const QGeoTilePos& tilePos);
  void removeWhenCovered(const QGeoTilePos& tilePos);
  void addTile(const QGeoTilePos& tilePos, QGraphicsItem * tileObj);
  void removeTile(const QGeoTilePos& tilePos);
  bool isTileExists(const QGeoTilePos& tilePos) const;
  bool isTileFinished(const QGeoTilePos& tilePos) const;
  QList<QGeoTilePos> existingTiles(int zoom) const;

protected:
  int currentZoom_ = -1;
  QRect currentRect_;
  QMap<int, QMap<QGeoTilePos, QGraphicsItem*>> mIndex;
  QElapsedTimer mLastAnimation;
};

#endif /* __QGeoTiles_h__ */
