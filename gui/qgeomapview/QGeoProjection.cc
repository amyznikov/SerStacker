/*
 * QGeoProjection.cc
 *
 *  Created on: Nov 24, 2022
 *      Author: amyznikov
 *
 *  Inspired by https://github.com/AmonRaNet/QGeoView
 */

#include "QGeoProjection.h"

QGeoProjection::QGeoProjection(const QString & id, const QString & name, const QString & description) :
    id_(id),
    name_(name),
    description_(description)
{
}

const QString& QGeoProjection::id() const
{
  return id_;
}

const QString& QGeoProjection::name() const
{
  return name_;
}

const QString& QGeoProjection::description() const
{
  return description_;
}
