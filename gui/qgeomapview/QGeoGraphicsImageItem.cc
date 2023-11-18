/*
 * QGeoMapImageGraphicsItem.cc
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#include "QGeoGraphicsImageItem.h"

#include <core/debug.h>

QGeoImageItem::QGeoImageItem()
{
}

QGeoImageItem::QGeoImageItem(const QGeoPos & geoPos, const QImage & image,
    const QPoint & imageAnchor) :
    Base(geoPos, image, imageAnchor)
{
}

QGeoImageItem::QGeoImageItem(const QGeoRect & geoRect, const QImage & image,
    const QPoint & imageAnchor) :
    Base(geoRect, image, imageAnchor)
{
}

void QGeoImageItem::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  if( !image_.isNull() ) {

    Base::paint(painter, option, widget);

    if( ignoreTransformation_ ) {
      painter->drawImage(-imageAnchor_, image_);
    }
    else {
      painter->drawImage(projRect_, image_,
          QRectF(0, 0, image_.width(), image_.height()));
    }
  }
}

QGeoPixmapItem::QGeoPixmapItem()
{
}

QGeoPixmapItem::QGeoPixmapItem(const QGeoPos & geoPos, const QPixmap & image,
    const QPoint & imageAnchor) :
    Base(geoPos, image, imageAnchor)
{
}

QGeoPixmapItem::QGeoPixmapItem(const QGeoRect & geoRect, const QPixmap & image,
    const QPoint & imageAnchor) :
    Base(geoRect, image, imageAnchor)
{
}

void QGeoPixmapItem::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  if( !image_.isNull() ) {

    Base::paint(painter, option, widget);

    if( ignoreTransformation_ ) {
      painter->drawPixmap(-imageAnchor_, image_);
    }
    else {
      painter->drawPixmap(projRect_, image_,
          QRectF(0, 0, image_.width(), image_.height()));
    }
  }
}
