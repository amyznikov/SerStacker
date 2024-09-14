/*
 * QGeoMapImageGraphicsItem.cc
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 */

#include "QGeoGraphicsImageItem.h"

#include <core/debug.h>

QGeoImageItem::QGeoImageItem(QGraphicsItem *parent) :
  Base(parent)
{
}

QGeoImageItem::QGeoImageItem(const QGeoPos & geoPos, const QImage & image,
    const QPoint & imageAnchor, QGraphicsItem *parent) :
    Base(geoPos, image, imageAnchor, parent)
{
}

QGeoImageItem::QGeoImageItem(const QGeoRect & geoRect, const QImage & image,
    const QPoint & imageAnchor, QGraphicsItem *parent) :
    Base(geoRect, image, imageAnchor, parent)
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

QGeoPixmapItem::QGeoPixmapItem(QGraphicsItem *parent) :
    Base(parent)
{
}

QGeoPixmapItem::QGeoPixmapItem(const QGeoPos & geoPos, const QPixmap & image,
    const QPoint & imageAnchor, QGraphicsItem *parent) :
    Base(geoPos, image, imageAnchor, parent)
{
}

QGeoPixmapItem::QGeoPixmapItem(const QGeoRect & geoRect, const QPixmap & image,
    const QPoint & imageAnchor, QGraphicsItem *parent) :
    Base(geoRect, image, imageAnchor, parent)
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
