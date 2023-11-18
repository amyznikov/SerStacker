/*
 * QGeoPos.h
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#pragma once
#ifndef __QGeoPos_h__
#define __QGeoPos_h__

#include <QtCore/QtCore>
#include <core/proc/gps/gps.h>

class QGeoPos
{
public:
  typedef QGeoPos ThisClass;

  QGeoPos();

  QGeoPos(double latitude, double longitude);

  void setLatitude(double latitude_degrees);
  double latitude() const;

  void setLongitude(double longitude_degrees);
  double longitude() const;

  QString lonToString(const QString& format = "[+-]d") const;
  QString latToString(const QString& format = "[+-]d") const;

  static QString latToString(double latitude, const QString& format = "[+-]d");
  static QString lonToString(double longitude, const QString& format = "[+-]d");

  bool operator ==(const QGeoPos & rhs) const
  {
    return fabs(this->latitude_ - rhs.latitude_) <= FLT_EPSILON  &&
        fabs(this->longitude_ - rhs.longitude_) <= FLT_EPSILON;
  }

  bool operator !=(const QGeoPos & rhs) const
  {
    return fabs(this->latitude_ - rhs.latitude_) > FLT_EPSILON ||
        fabs(this->longitude_ - rhs.longitude_) > FLT_EPSILON;
  }

#if HAVE_c_gps_position

  QGeoPos(const c_gps_position & gps) :
      ThisClass(gps.latitude * 180 / M_PI, gps.longitude * 180 / M_PI)
  {
  }

  QGeoPos & setGpsPos(const c_gps_position & gps)
  {
    this->latitude_ = gps.latitude * 180 / M_PI;
    this->longitude_ = gps.longitude * 180 / M_PI;
    return *this;
  }

  QGeoPos & operator = (const c_gps_position & gps)
  {
    return setGpsPos(gps);
  }

  bool operator ==(const c_gps_position & rhs) const
  {
    return fabs(this->latitude_ - rhs.latitude * 180 / M_PI) <= FLT_EPSILON &&
        fabs(this->longitude_ - rhs.longitude * 180 / M_PI) <= FLT_EPSILON;
  }

  bool operator !=(const c_gps_position & rhs) const
  {
    return fabs(this->latitude_ - rhs.latitude * 180 / M_PI) > FLT_EPSILON ||
        fabs(this->longitude_ - rhs.longitude * 180 / M_PI) > FLT_EPSILON;
  }

  void getGps(c_gps_position & gps) const
  {
    gps.latitude = this->latitude_ * M_PI / 180;
    gps.longitude = this->longitude_ * M_PI / 180;
  }


#endif

private:
  double latitude_;
  double longitude_;
};

Q_DECLARE_METATYPE(QGeoPos);

#endif /* __QGeoPos_h__ */
