/*
 * QGeoPos.cc
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoPos.h"

bool QGeoPos::registerMetatype()
{
  qRegisterMetaTypeStreamOperators<QGeoPos>("QGeoPos");
  return true;
}

const bool QGeoPos::_metatypeRegistered =
    QGeoPos::registerMetatype();

QGeoPos::QGeoPos() :
  _latitude(0),
  _longitude(0)
{
}

QGeoPos::QGeoPos(double latitude, double longitude)
{
  setLatitude(latitude);
  setLongitude(longitude);
}


QDataStream& operator <<(QDataStream & arch, const QGeoPos & gpos)
{
  return (arch << gpos._latitude << gpos._longitude);
}

QDataStream& operator >>(QDataStream & arch, QGeoPos & gpos)
{
  return (arch >> gpos._latitude >> gpos._longitude);
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
