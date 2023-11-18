/*
 * QGeoMapOnlineTilesLayer.h
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoMapOnlineTilesLayer_h__
#define __QGeoMapOnlineTilesLayer_h__

#include "QGeoMapOnlineItem.h"
#include "QGeoTiles.h"

class QGeoMapOnlineTiles:
    public QGeoTiles,
    public QGeoMapOnlineItem
{
  Q_OBJECT;
public:
  typedef QGeoMapOnlineTiles ThisClass;
  typedef QGeoTiles Base;

  QGeoMapOnlineTiles();
  ~QGeoMapOnlineTiles();

  virtual QString tilePosToUrl(const QGeoTilePos& tilePos) const = 0;

protected:
  void request(const QGeoTilePos& tilePos) override;
  void cancel(const QGeoTilePos& tilePos) override;
  void onReplyFinished(QNetworkReply* reply);
  void removeReply(const QGeoTilePos& tilePos);

private:
  QMap<QGeoTilePos, QNetworkReply*> mRequest;
};

#endif /* __QGeoMapOnlineTilesLayer_h__ */
