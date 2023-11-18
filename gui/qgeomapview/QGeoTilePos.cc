/*
 * QGeoTilePos.cc
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoTilePos.h"

QGeoTilePos::QGeoTilePos() :
  zoom_(-1)
{
}


QGeoTilePos::QGeoTilePos(int zoom, const QPoint & pos) :
    zoom_(zoom),
    pos_(pos)
{
}

bool QGeoTilePos::operator<(const QGeoTilePos & other) const
{
  if( zoom_ < other.zoom_ ) {
    return true;
  }
  if( zoom_ > other.zoom_ ) {
    return false;
  }
  if( pos_.x() < other.pos_.x() ) {
    return true;
  }
  if( pos_.x() > other.pos_.x() ) {
    return false;
  }
  return pos_.y() < other.pos_.y();
}

int QGeoTilePos::zoom() const
{
  return zoom_;
}

const QPoint & QGeoTilePos::pos() const
{
 return pos_;
}

bool QGeoTilePos::contains(const QGeoTilePos & other) const
{
  if( zoom_ >= other.zoom_ ) {
    return false;
  }

  const QGeoTilePos parentTile =
      other.parent(zoom_);

  return (pos_.x() == parentTile.pos_.x() &&
      pos().y() == parentTile.pos_.y());
}

QGeoTilePos QGeoTilePos::parent(int parentZoom) const
{
  if( parentZoom >= zoom_ ) {
    return QGeoTilePos();
  }

  const int deltaZoom =
      zoom_ - parentZoom;

  const int factor =
      static_cast<int>(qPow(2, deltaZoom));

  const int x =
      static_cast<int>(qFloor(pos_.x() / factor));

  const int y =
      static_cast<int>(qFloor(pos_.y() / factor));

  return QGeoTilePos(parentZoom,
      QPoint(x, y));
}

QGeoRect QGeoTilePos::toGeoRect() const
{
  const auto leftTop =
      [](const QGeoTilePos & tilePos) -> QGeoPos {

        const int zoom =
            tilePos.zoom();

        const int x =
            tilePos.pos().x();

        const int y =
            tilePos.pos().y();

        const double lon =
            x / pow(2.0, zoom) * 360.0 - 180;

        const double n =
            M_PI - 2.0 * M_PI * y / pow(2.0, zoom);

        const double lat =
            180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));

        return QGeoPos(lat, lon);
      };

  const QGeoPos pos1 =
      leftTop(*this);

  const QGeoPos pos2 =
      leftTop(QGeoTilePos(zoom_,
          pos_ + QPoint(1, 1)));

  return QGeoRect(pos1, pos2);
}

QString QGeoTilePos::toQuadKey() const
{
  const int x =
      pos_.x();

  const int y =
      pos_.y();

  QString quadKey;

  for( int i = zoom_; i > 0; i-- ) {

    char cDigit = '0';

    int iMask = 1 << (i - 1);

    if( (x & iMask) != 0 ) {
      cDigit++;
    }

    if( (y & iMask) != 0 ) {
      cDigit++;
      cDigit++;
    }

    quadKey.append(cDigit);
  }

  return quadKey;
}

QGeoTilePos QGeoTilePos::geoToTilePos(int zoom, const QGeoPos & geoPos)
{
  const double lon =
      geoPos.longitude();

  const double lat =
      geoPos.latitude();

  const double x =
      floor((lon + 180.0) / 360.0 * pow(2.0, zoom));

  const double y =
      floor((1.0 - log(tan(lat * M_PI / 180.0) + 1.0 / cos(lat * M_PI / 180.0)) / M_PI) / 2.0 * pow(2.0, zoom));

  return QGeoTilePos(zoom,
      QPoint(static_cast<int>(x),
          static_cast<int>(y)));
}
