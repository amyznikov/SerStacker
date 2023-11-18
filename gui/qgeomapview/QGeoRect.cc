/*
 * QGeoRect.cc
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoRect.h"

QGeoRect::QGeoRect()
{
}

QGeoRect::QGeoRect(double lat1, double lon1, double lat2, double lon2) :
    topLeft_(QGeoPos(qMax(lat1, lat2), qMin(lon1, lon2))),
    bottomRight_(QGeoPos(qMin(lat1, lat2), qMax(lon1, lon2)))
{
}

QGeoRect::QGeoRect(const QGeoPos & pos1, const QGeoPos & pos2) :
    topLeft_(QGeoPos(qMax(pos1.latitude(), pos2.latitude()), qMin(pos1.longitude(), pos2.longitude()))),
    bottomRight_(QGeoPos(qMin(pos1.latitude(), pos2.latitude()), qMax(pos1.longitude(), pos2.longitude())))
{
}

const QGeoPos& QGeoRect::topLeft() const
{
  return topLeft_;
}

const QGeoPos& QGeoRect::bottomRight() const
{
  return bottomRight_;
}

QGeoPos QGeoRect::bottomLeft() const
{
  return QGeoPos(bottomRight_.latitude(), topLeft_.longitude());
}

QGeoPos QGeoRect::topRight() const
{
  return QGeoPos(topLeft_.latitude(), bottomRight_.longitude());
}

QGeoPos QGeoRect::center() const
{
  return QGeoPos(0.5 * (topLeft_.latitude() + bottomRight_.latitude()),
      0.5 * (topLeft_.longitude() + bottomRight_.longitude()));
}

double QGeoRect::lonLeft() const
{
  return topLeft_.longitude();
}

double QGeoRect::lonRigth() const
{
  return bottomRight_.longitude();
}

double QGeoRect::latBottom() const
{
  return bottomRight_.latitude();
}

double QGeoRect::latTop() const
{
  return topLeft_.latitude();
}

bool QGeoRect::contains(QGeoPos const & pos) const
{
  return (lonLeft() <= pos.longitude() &&
      pos.longitude() < lonRigth() &&
      latBottom() < pos.latitude() &&
      pos.latitude() <= latTop());
}

bool QGeoRect::contains(QGeoRect const & rect) const
{
  return (lonLeft() <= rect.lonLeft() &&
      rect.lonRigth() <= lonRigth() &&
      latBottom() <= rect.latBottom() &&
      rect.latTop() <= latTop());
}

bool QGeoRect::intersects(QGeoRect const & rect) const
{
  return contains(rect.topLeft()) ||
      contains(rect.topRight()) ||
      contains(rect.bottomLeft()) ||
      contains(rect.bottomRight()) ||
      rect.contains(topLeft()) ||
      rect.contains(topRight()) ||
      rect.contains(bottomLeft()) ||
      rect.contains(bottomRight());
}
