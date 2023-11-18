/*
 * QGeoViewCameraState.h
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoViewCameraState_h__
#define __QGeoViewCameraState_h__

#include <QtCore/QtCore>

class QGeoView;
class QGeoProjection;

class QGeoViewCameraState
{
public:
  typedef QGeoViewCameraState ThisClass;

  QGeoViewCameraState(QGeoView * geoView,
      double azimuth,
      double scale,
      const QRectF & projRect,
      bool animation);

  //QGeoViewCameraState(const QGeoViewCameraState & rhs) = default;

  QGeoView* view() const;
  QGeoProjection* projection() const;

  double scale() const;
  double azimuth() const;
  const QRectF & projRect() const;
  QPointF projCenter() const;
  bool animation() const;

  bool operator==(const QGeoViewCameraState & other) const;
  bool operator!=(const QGeoViewCameraState & other) const;

private:
  QGeoView *view_;
  double scale_;
  double azimuth_;
  QRectF projRect_;
  bool animation_;
};


#endif /* __QGeoViewCameraState_h__ */
