/*
 * QGeoTilesGoogle.h
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoTilesGoogle_h__
#define __QGeoTilesGoogle_h__

#include "QGeoMapOnlineTiles.h"

class QGeoTilesGoogle:
    public QGeoMapOnlineTiles
{
public:
  typedef QGeoTilesGoogle ThisClass;
  typedef QGeoMapOnlineTiles Base;

  enum class TilesType {
      Satellite,
      Schema,
      Hybrid,
  };

  QGeoTilesGoogle(TilesType type = TilesType::Schema,
      const QLocale & locale = QLocale(),
      int serverNumber = 0);

  void setTilesType(TilesType type);
  TilesType tilesType() const;

  void setLocale(const QLocale& locale);
  const QLocale & locale() const;

protected:
  void updateName();
  int minZoomlevel() const override;
  int maxZoomlevel() const override;
  QString tilePosToUrl(const QGeoTilePos& tilePos) const override;

protected:
  TilesType tilesType_;
  QLocale locale_;
  int serverNumber_;
};

#endif /* __QGeoTilesGoogle_h__ */
