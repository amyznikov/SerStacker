/*
 * QGeoTilePos.h
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#pragma once
#ifndef __QGeoTilePos_h__
#define __QGeoTilePos_h__

#include "QGeoPos.h"
#include "QGeoRect.h"

class QGeoTilePos
{
public:
  typedef QGeoTilePos ThisClass;

  QGeoTilePos();
  QGeoTilePos(int zoom, const QPoint& pos);

  bool operator<(const QGeoTilePos& other) const;

  int zoom() const;
  const QPoint & pos() const;

  bool contains(const QGeoTilePos& other) const;
  QGeoTilePos parent(int parentZoom) const;

  QGeoRect toGeoRect() const;
  QString toQuadKey() const;

  static QGeoTilePos geoToTilePos(int zoom,
      const QGeoPos& geoPos);

private:
  int zoom_;
  QPoint pos_;
};

Q_DECLARE_METATYPE(QGeoTilePos);

#endif /* __QGeoTilePos_h__ */
