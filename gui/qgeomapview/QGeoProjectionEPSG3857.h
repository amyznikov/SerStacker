/*
 * QGeoProjectionEPSG3857.h
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Copied from https://github.com/AmonRaNet/QGeoView
 */

#pragma once
#ifndef __QGeoProjectionEPSG3857_h__
#define __QGeoProjectionEPSG3857_h__

#include "QGeoProjection.h"

class QGeoProjectionEPSG3857:
    public QGeoProjection
{
public:
  typedef QGeoProjectionEPSG3857 ThisClass;
  typedef QGeoProjection Base;

  QGeoProjectionEPSG3857();

  QGeoRect boundaryGeoRect() const override;
  QRectF boundaryProjRect() const override;

  QPointF geoToProj(const QGeoPos & geoPos) const override;
  QGeoPos projToGeo(const QPointF & projPos) const override;
  QRectF geoToProj(const QGeoRect & geoRect) const override;
  QGeoRect projToGeo(const QRectF & projRect) const override;
  double geodesicMeters(const QPointF & projPos1, const QPointF & projPos2) const override;

protected:
  QGeoRect geoBoundary_;
  QRectF projBoundary_;
};

#endif /* __QGeoProjectionEPSG3857_h__ */
