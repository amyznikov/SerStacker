/*
 * QGeoGraphicsImageItem.h
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 *
 *  // To Find the active view for item's mouse event:
 *  QGraphicsView *view = nullptr;
 *  if (event->widget())
 *    view = qobject_cast<QGraphicsView *>(event->widget()->parentWidget());
 *
 */

#pragma once
#ifndef __QGeoGraphicsImageItem_h__
#define __QGeoGraphicsImageItem_h__

#include "QGeoPos.h"
#include "QGeoRect.h"
#include "QGeoGraphicsItem.h"

template<class ImageType>
class QGeoGraphicsImageItem:
    public QGeoGraphicsItem
{
public:
  typedef QGeoGraphicsImageItem ThisClass;
  typedef QGeoGraphicsItem Base;

  QGeoGraphicsImageItem(QGraphicsItem *parent = nullptr) :
    Base(parent)
  {
  }

  QGeoGraphicsImageItem(const QGeoPos & geoPos, const ImageType & image, const QPoint & imageAnchor = QPoint(-1, -1),
      QGraphicsItem * parent = nullptr) :
      Base(parent)
  {
    setImage(geoPos, image, imageAnchor);
  }

  QGeoGraphicsImageItem(const QGeoRect & geoRect, const ImageType & image, const QPoint & imageAnchor = QPoint(-1, -1),
      QGraphicsItem * parent = nullptr) :
      Base(parent)
  {
    setImage(geoRect, image, imageAnchor);
  }

  void setImage(const ImageType & image)
  {
    const bool needUpdateGeometry =
        geometryType_ != GeometryType::None &&
        imageSize_ != image.size();

    imageSize_ =
        (image_ = image).size();

    if ( needUpdateGeometry ) {
      updateProjected();
    }
  }

  void setImage(const QGeoPos & geoPos, const ImageType & image,
      const QPoint & imageAnchor = QPoint(-1, -1))
  {
    setGeometry(geoPos, (image_ = image).size(), imageAnchor);
  }

  void setImage(const QGeoRect & geoRect, const ImageType & image,
      const QPoint & imageAnchor = QPoint(-1, -1))
  {
    setGeometry(geoRect, (image_ = image).size(), imageAnchor);
  }

  const ImageType & image() const
  {
    return image_;
  }

  void setGeometry(const QGeoPos & geoPos, const QSize & imageSize = QSize(-1, -1),
      const QPoint & imageAnchor = QPoint(-1, -1))
  {
    geometryType_ = GeometryType::ByPos;
    imageSize_ = imageSize;
    geoPos_ = geoPos;

    if( imageAnchor.x() < 0 || imageAnchor.y() < 0 ) {
      imageAnchor_ =
          QPoint(imageSize_.width() / 2,
              imageSize_.height() / 2);
    }
    else {
      imageAnchor_ =
          imageAnchor;
    }

    updateProjected();
  }

  void setGeometry(const QGeoRect & geoRect, const QSize & imageSize = QSize(-1, -1),
      const QPoint & imageAnchor = QPoint(-1, -1))
  {
    geometryType_ = GeometryType::ByRect;
    imageSize_ = imageSize;
    geoRect_ = geoRect;

    if( imageAnchor.x() < 0 || imageAnchor.y() < 0 ) {
      imageAnchor_ =
          QPoint(imageSize_.width() / 2,
              imageSize_.height() / 2);
    }
    else {
      imageAnchor_ =
          imageAnchor;
    }

    updateProjected();
  }

  void setGeoPos(const QGeoPos & geopos)
  {
    geometryType_ = GeometryType::ByPos;
    geoPos_ = geopos;
    updateProjected();
  }

  const QGeoPos & geoPos() const
  {
    return geoPos_;
  }

protected:

  void updateProjected(const QGeoScene * scene = nullptr) override
  {
    // The Geo coordinates are assumed to be valid on entry,
    // projected coordinates must be updated

    if( scene || (scene = geoScene()) ) {

      switch (geometryType_) {
      case GeometryType::ByPos: {

        QGeoProjection * projection =
            scene->projection();

        const QPointF basePos =
            projection->geoToProj(geoPos_);

        projRect_ =
            QRectF(basePos - imageAnchor_,
                imageSize_);

        geoRect_ =
            projection->projToGeo(projRect_);

        projAnchor_ =
            basePos;

        break;
      }

      case GeometryType::ByRect: {

        QGeoProjection * projection =
            scene->projection();

        projRect_ =
            projection->geoToProj(geoRect_);

        projAnchor_.setX(projRect_.x() +
            (imageSize_.width() > 0 ?
                projRect_.width() * imageAnchor_.x() / imageSize_.width() :
                projRect_.width() / 2));

        projAnchor_.setY(projRect_.y() +
            (imageSize_.height() > 0 ?
                projRect_.height() * imageAnchor_.y() / imageSize_.height() :
                projRect_.height() / 2));

        geoPos_ =
            projection->projToGeo(projAnchor_);

        break;
      }

      default:
        break;
      }

      updateLocalPos();
    }

    onGeoPosChanged(geoPos_);
  }

  void updateGeo(const QGeoScene * scene) override
  {
    // The projected coordinates was changed and are assumed to be valid on entry,
    // Geo coordinates must be updated

    if( scene || (scene = geoScene()) ) {

      QPointF scenePos =
          this->scenePos();

      projAnchor_ = scenePos;
      projRect_ = QRectF(projAnchor_ - imageAnchor_, imageSize_);

      const QGeoProjection * projection =
          scene->projection();

      if ( projection ) {
        geoPos_ = projection->projToGeo(projAnchor_);
        geoRect_ = projection->projToGeo(projRect_);
      }

      if ( !ignoreTransformation_ ) {
        updateLocalPos();
      }

      if ( projection ) {
        onGeoPosChanged(geoPos_);
      }
    }
  }


  void updateLocalPos()
  {
    // Both projected and Geo data are assumed to be valid on this input.
    // The this->pos() is the position of the item in parent coordinates.
    // If the item has no parent, its position is given in scene coordinates.

    prepareGeometryChange();

    setUpdatingPos(true);

    if( !ignoreTransformation_ ) {

      const QGraphicsItem *parent =
          parentItem();

      setPos(parent ? projAnchor_ - parent->scenePos() : projAnchor_);
      projRect_.translate(-scenePos());
    }
    else {

      setPos(0, 0);

      const QGraphicsItem *parent =
          ignoreTransformation_->parentItem();

      ignoreTransformation_->setPos(parent ?
          projAnchor_ - parent->scenePos() :
          projAnchor_);
    }

    setUpdatingPos(false);
  }

protected: // QGraphicsObject
  // the outer bounds of the item as a rectangle in local coordinates
  QRectF boundingRect() const override
  {
    return ignoreTransformation_ ?
        QRectF(-imageAnchor_.x(), -imageAnchor_.y(), imageSize_.width(), imageSize_.height()) :
        projRect_;
  }

  // Returns the shape of this item as a QPainterPath in local coordinates.
  QPainterPath shape() const override
  {
    QPainterPath path;

    if ( ignoreTransformation_ ) {
      path.addRect(-imageAnchor_.x(), -imageAnchor_.y(),
          imageSize_.width(), imageSize_.height());
    }
    else {
      path.addRect(projRect_);
    }

    return path;
  }

protected:
  enum class GeometryType {
    None,
    ByRect,
    ByPos,
  } geometryType_ = GeometryType::None;

  ImageType image_;
  QSize imageSize_;
  QGeoRect geoRect_;
  QGeoPos geoPos_;
  QRectF projRect_;
  QPoint imageAnchor_;
  QPointF projAnchor_;
};

class QGeoImageItem:
    public QGeoGraphicsImageItem<QImage>
{
public:
  typedef QGeoImageItem ThisClass;
  typedef QGeoGraphicsImageItem<QImage> Base;

  QGeoImageItem(QGraphicsItem *parent = nullptr);

  QGeoImageItem(const QGeoPos & geoPos, const QImage & image,
      const QPoint & imageAnchor = QPoint(-1, -1),
      QGraphicsItem *parent = nullptr);

  QGeoImageItem(const QGeoRect & geoRect, const QImage & image,
      const QPoint & imageAnchor = QPoint(-1, -1),
      QGraphicsItem *parent = nullptr);

protected:
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option,
      QWidget * widget = nullptr) override;
};

class QGeoPixmapItem:
    public QGeoGraphicsImageItem<QPixmap>
{
public:
  typedef QGeoPixmapItem ThisClass;
  typedef QGeoGraphicsImageItem<QPixmap> Base;

  QGeoPixmapItem(QGraphicsItem *parent = nullptr);

  QGeoPixmapItem(const QGeoPos & geoPos, const QPixmap & image,
      const QPoint & imageAnchor = QPoint(-1, -1),
      QGraphicsItem *parent = nullptr);

  QGeoPixmapItem(const QGeoRect & geoRect, const QPixmap & image,
      const QPoint & imageAnchor = QPoint(-1, -1),
      QGraphicsItem *parent = nullptr);

protected:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
      QWidget *widget = nullptr) override;
};

#endif /* __QGeoGraphicsImageItem_h__ */
