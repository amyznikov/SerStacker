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

  void setLatitude(double latitude)
  {
    _latitude = qMin(90.0, qMax(-90.0, latitude));
  }

  double latitude() const
  {
    return _latitude;
  }


  void setLongitude(double longitude)
  {
    if( longitude > 180.000001 ) {
      longitude = fmod((180.0 + longitude), 360.0) - 180.0;
    }
    else if( longitude < -180.000001 ) {
      longitude = 180.0 - fmod((180.0 - longitude), 360.0);
    }

    _longitude = longitude;
  }

  double longitude() const
  {
    return _longitude;
  }

  QString lonToString(const QString& format = "[+-]d") const
  {
    return lonToString(longitude(), format);
  }

  QString latToString(const QString& format = "[+-]d") const
  {
    return latToString(latitude(), format);
  }

  bool operator ==(const QGeoPos & rhs) const
  {
    return fabs(this->_latitude - rhs._latitude) <= FLT_EPSILON  &&
        fabs(this->_longitude - rhs._longitude) <= FLT_EPSILON;
  }

  bool operator !=(const QGeoPos & rhs) const
  {
    return fabs(this->_latitude - rhs._latitude) > FLT_EPSILON ||
        fabs(this->_longitude - rhs._longitude) > FLT_EPSILON;
  }

  static QString latToString(double latitude, const QString& format = "[+-]d");
  static QString lonToString(double longitude, const QString& format = "[+-]d");

#if HAVE_c_gps_position

  QGeoPos(const c_gps_position & gps) :
      ThisClass(gps.latitude * 180 / M_PI, gps.longitude * 180 / M_PI)
  {
  }

  QGeoPos & setGpsPos(const c_gps_position & gps)
  {
    this->_latitude = gps.latitude * 180 / M_PI;
    this->_longitude = gps.longitude * 180 / M_PI;
    return *this;
  }

  QGeoPos & operator = (const c_gps_position & gps)
  {
    return setGpsPos(gps);
  }

  bool operator ==(const c_gps_position & rhs) const
  {
    return fabs(this->_latitude - rhs.latitude * 180 / M_PI) <= FLT_EPSILON &&
        fabs(this->_longitude - rhs.longitude * 180 / M_PI) <= FLT_EPSILON;
  }

  bool operator !=(const c_gps_position & rhs) const
  {
    return fabs(this->_latitude - rhs.latitude * 180 / M_PI) > FLT_EPSILON ||
        fabs(this->_longitude - rhs.longitude * 180 / M_PI) > FLT_EPSILON;
  }

  void getGps(c_gps_position & gps) const
  {
    gps.latitude = this->_latitude * M_PI / 180;
    gps.longitude = this->_longitude * M_PI / 180;
  }


#endif

private:
  double _latitude;
  double _longitude;

private:
  static bool registerMetatype();
  static const bool _metatypeRegistered;
  friend QDataStream& operator <<(QDataStream & arch, const QGeoPos & gpos);
  friend QDataStream& operator >>(QDataStream & arch, QGeoPos & gpos);
};

Q_DECLARE_METATYPE(QGeoPos);


#endif /* __QGeoPos_h__ */
