/*
 * QGeoProjectionEPSG3857.cc
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Copied from https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoProjectionEPSG3857.h"

static constexpr double earthRadius = 6378137.0; /* meters */;
static constexpr double originShift = 2.0 * M_PI * earthRadius / 2.0;


QGeoProjectionEPSG3857::QGeoProjectionEPSG3857() :
    Base("EPSG3857",
        "WGS84 Web Mercator",
        "QGVProjection used in web mapping applications like "
            "Google/Bing/OpenStreetMap/etc. Sometimes known as "
            "EPSG:900913.")

{
  geoBoundary_ = QGeoRect(85, -180, -85, +180);
  projBoundary_ = geoToProj(geoBoundary_);
}


QGeoRect QGeoProjectionEPSG3857::boundaryGeoRect() const
{
  return geoBoundary_;
}

QRectF QGeoProjectionEPSG3857::boundaryProjRect() const
{
  return projBoundary_;
}


QPointF QGeoProjectionEPSG3857::geoToProj(const QGeoPos & geoPos) const
{
  const double lon =
      geoPos.longitude();

  const double lat =
      (geoPos.latitude() > geoBoundary_.latTop()) ?
          geoBoundary_.latTop() :
          geoPos.latitude();

  const double x =
      lon * originShift / 180.0;

  const double preY =
      -qLn(qTan((90.0 + lat) * M_PI / 360.0)) / (M_PI / 180.0);

  const double y =
      preY * originShift / 180.0;

  return QPointF(x, y);
}

QGeoPos QGeoProjectionEPSG3857::projToGeo(QPointF const& projPos) const
{
  const double lon =
      (projPos.x() / originShift) * 180.0;

  const double preLat =
      (-projPos.y() / originShift) * 180.0;

  const double lat =
      180.0 / M_PI * (2.0 * qAtan(qExp(preLat * M_PI / 180.0)) - M_PI / 2.0);

  return QGeoPos(lat, lon);
}

QRectF QGeoProjectionEPSG3857::geoToProj(const QGeoRect & geoRect) const
{
  return QRectF(geoToProj(geoRect.topLeft()),
      geoToProj(geoRect.bottomRight()));
}

QGeoRect QGeoProjectionEPSG3857::projToGeo(QRectF const& projRect) const
{
  return QGeoRect(projToGeo(projRect.topLeft()),
      projToGeo(projRect.bottomRight()));
}

double QGeoProjectionEPSG3857::geodesicMeters(QPointF const & projPos1, QPointF const & projPos2) const
{
  const QGeoPos geoPos1 =
      projToGeo(projPos1);

  const QGeoPos geoPos2 =
      projToGeo(projPos2);

  const double latitudeArc =
      (geoPos1.latitude() - geoPos2.latitude()) * M_PI / 180.0;

  const double longitudeArc =
      (geoPos1.longitude() - geoPos2.longitude()) * M_PI / 180.0;

  const double latitudeH =
      qPow(sin(latitudeArc * 0.5), 2);

  const double lontitudeH =
      qPow(sin(longitudeArc * 0.5), 2);

  const double lonFactor =
      cos(geoPos1.latitude() * M_PI / 180.0) * cos(geoPos2.latitude() * M_PI / 180.0);

  const double arcInRadians =
      2.0 * asin(sqrt(latitudeH + lonFactor * lontitudeH));

  return earthRadius * arcInRadians;
}

