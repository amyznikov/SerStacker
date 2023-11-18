/*
 * QGeoPos.cc
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoPos.h"

QGeoPos::QGeoPos() :
  latitude_(0),
  longitude_(0)
{
}


QGeoPos::QGeoPos(double latitude, double longitude)
{
  setLatitude(latitude);
  setLongitude(longitude);
}

void QGeoPos::setLatitude(double latitude)
{
  latitude_ = qMin(90.0, qMax(-90.0, latitude));
}

double QGeoPos::latitude() const
{
  return latitude_;
}

void QGeoPos::setLongitude(double longitude)
{
  if( longitude > 180.000001 ) {
    longitude = fmod((180.0 + longitude), 360.0) - 180.0;
  }
  else if( longitude < -180.000001 ) {
    longitude = 180.0 - fmod((180.0 - longitude), 360.0);
  }

  longitude_ = longitude;
}

double QGeoPos::longitude() const
{
  return longitude_;
}


QString QGeoPos::lonToString(const QString& format) const
{
  return lonToString(longitude(), format);
}

QString QGeoPos::latToString(const QString& format) const
{
  return latToString(latitude(), format);
}

/**
 * Longitude to string
 * Format:
 * [+-] - sign
 * d -  degree (unsigned)
 * di - degree integer part only(unsigned)
 * m -  minute only(unsigned)
 * mi - minute integer part only(unsigned)
 * s -  second (unsigned)
 * si - second integer part only(unsigned)
 * \brief lonToString
 * \param lon
 * \param aFormat
 * \return
 */
QString QGeoPos::lonToString(double lon, const QString& format)
{
  QString result = format;

  const QString signSymb = (lon < 0) ? "-" : QString();
  const double degreePart = qAbs(lon);
  const double minPart = (degreePart - static_cast<int>(degreePart)) * 60.0;
  const double secPart = (minPart - static_cast<int>(minPart)) * 60.0;

  result.replace("[+-]", signSymb);
  result.replace("di", QString::number(static_cast<int>(degreePart)));
  result.replace("d", QString::number(degreePart, 'f', 6));
  result.replace("mi", QString::number(static_cast<int>(minPart)));
  result.replace("m", QString::number(minPart, 'f', 4));
  result.replace("si", QString::number(static_cast<int>(secPart)));
  result.replace("s", QString::number(secPart, 'f', 3));

  return result;
}

/**
 * Latitude to string
 * Format:
 * [+-] - sign
 * [NS] - North or South
 * d -  degree (unsigned)
 * di - degree integer part only(unsigned)
 * m -  minute only(unsigned)
 * mi - minute integer part only(unsigned)
 * s -  second (unsigned)
 * si - second integer part only(unsigned)
 */
QString QGeoPos::latToString(double lat, const QString& format)
{
  QString result = format;

  const QString signSymb = (lat < 0) ? "-" : QString();
  const QString northSouth = (lat < 0) ? "N" : "S";
  const double degreePart = qAbs(lat);
  const double minPart = (degreePart - static_cast<int>(degreePart)) * 60.0;
  const double secPart = (minPart - static_cast<int>(minPart)) * 60.0;

  result.replace("[+-]", signSymb);
  result.replace("[NS]", northSouth);
  result.replace("di", QString::number(static_cast<int>(degreePart)));
  result.replace("d", QString::number(degreePart, 'f', 6));
  result.replace("mi", QString::number(static_cast<int>(minPart)));
  result.replace("m", QString::number(minPart, 'f', 4));
  result.replace("si", QString::number(static_cast<int>(secPart)));
  result.replace("s", QString::number(secPart, 'f', 3));

  return result;
}
