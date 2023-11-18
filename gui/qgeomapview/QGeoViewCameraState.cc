/*
 * QGeoViewCameraState.cc
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#include "QGeoViewCameraState.h"
#include "QGeoView.h"

QGeoViewCameraState::QGeoViewCameraState(QGeoView * geoView, double azimuth, double scale,
    const QRectF & projRect, bool animation) :
    view_(geoView),
    azimuth_(azimuth),
    scale_(scale),
    projRect_(projRect),
    animation_(animation)
{
}

QGeoView* QGeoViewCameraState::view() const
{
  return view_;
}

QGeoProjection* QGeoViewCameraState::projection() const
{
  return view_ ? view_->projection() : nullptr;
}

double QGeoViewCameraState::scale() const
{
  return scale_;
}

double QGeoViewCameraState::azimuth() const
{
  return azimuth_;
}

const QRectF & QGeoViewCameraState::projRect() const
{
  return projRect_;
}

QPointF QGeoViewCameraState::projCenter() const
{
  return projRect_.center();
}

bool QGeoViewCameraState::animation() const
{
  return animation_;
}

bool QGeoViewCameraState::operator==(const QGeoViewCameraState & other) const
{
  return view_ == other.view_ &&
      qFuzzyCompare(scale_, other.scale_) &&
      qFuzzyCompare(azimuth_, other.azimuth_) &&
      projRect_ == other.projRect_ &&
      animation_ == other.animation_;
}

bool QGeoViewCameraState::operator!=(const QGeoViewCameraState & other) const
{
  return !(*this == other);
}


