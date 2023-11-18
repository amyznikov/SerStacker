/*
 * QGeoProjection.h
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#pragma once
#ifndef __QGeoProjection_h__
#define __QGeoProjection_h__

#include "QGeoPos.h"
#include "QGeoRect.h"

enum class QGeoProjectionType
{
  QGeoProjection_EPSG3857,
};


class QGeoProjection
{
public:
  typedef QGeoProjection ThisClass;

  QGeoProjection(const QString & id, const QString & name,
      const QString & description);

  virtual ~QGeoProjection() = default;

  const QString& id() const;
  const QString& name() const;
  const QString& description() const;

  virtual QGeoRect boundaryGeoRect() const = 0;
  virtual QRectF boundaryProjRect() const = 0;

  virtual QPointF geoToProj(QGeoPos const & geoPos) const = 0;
  virtual QGeoPos projToGeo(QPointF const & projPos) const = 0;
  virtual QRectF geoToProj(QGeoRect const & geoRect) const = 0;
  virtual QGeoRect projToGeo(QRectF const & projRect) const = 0;
  virtual double geodesicMeters(QPointF const & projPos1, QPointF const & projPos2) const = 0;

private:
  Q_DISABLE_COPY(QGeoProjection);
  QString id_;
  QString name_;
  QString description_;
};

#endif /* __QGeoProjection_h__ */
