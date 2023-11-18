/*
 * QGeoRect.h
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#pragma once
#ifndef __QGeoRect_h__
#define __QGeoRect_h__

#include "QGeoPos.h"

class QGeoRect
{
public:
  typedef QGeoRect ThisClass;

  QGeoRect();

  QGeoRect(double lat1, double lon1, double lat2, double lon2);
  QGeoRect(const QGeoPos & pos1, const QGeoPos & pos2);

#if HAVE_c_gps_position

  QGeoRect(const c_gps_position & gps1, const c_gps_position & gps2) :
    ThisClass(QGeoPos(gps1), QGeoPos(gps2) )
  {
  }

#endif // HAVE_c_gps_position

  const QGeoPos& topLeft() const;
  const QGeoPos& bottomRight() const;
  QGeoPos bottomLeft() const;
  QGeoPos topRight() const;
  QGeoPos center() const;
  double lonLeft() const;
  double lonRigth() const;
  double latBottom() const;
  double latTop() const;

  bool contains(QGeoPos const & pos) const;
  bool contains(QGeoRect const & rect) const;
  bool intersects(QGeoRect const & rect) const;

private:
  QGeoPos topLeft_;
  QGeoPos bottomRight_;
};

Q_DECLARE_METATYPE(QGeoRect);

#endif /* __QGeoRect_h__ */
