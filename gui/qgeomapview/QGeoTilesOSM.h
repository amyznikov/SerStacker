/*
 * QGeoTilesOSM.h
 *
 *  Created on: Nov 29, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoTilesOSM_h__
#define __QGeoTilesOSM_h__

#include "QGeoMapOnlineTiles.h"

class QGeoTilesOSM:
    public QGeoMapOnlineTiles
{
public:
  typedef QGeoTilesOSM ThisClass;
  typedef QGeoMapOnlineTiles Base;

  QGeoTilesOSM(int serverNumber = 0);
  QGeoTilesOSM(const QString & url);

  void setUrl(const QString & url);
  const QString& getUrl() const;

protected:
  int minZoomlevel() const override;
  int maxZoomlevel() const override;
  QString tilePosToUrl(const QGeoTilePos & tilePos) const override;

protected:
  QString url_;
};

#endif /* __QGeoTilesOSM_h__ */
