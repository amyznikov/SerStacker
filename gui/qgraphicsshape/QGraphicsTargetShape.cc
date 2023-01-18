/*
 * QGraphicsTargetShape.cc
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#include "QGraphicsTargetShape.h"
#include <core/debug.h>

QGraphicsTargetShape::QGraphicsTargetShape(QGraphicsItem * parent) :
  Base(parent)
{
  setCosmeticPen(Qt::red, 1);
}

void QGraphicsTargetShape::setCenter(const QPointF & v)
{
  if( center_ != v ) {
    prepareGeometryChange();
    center_ = v;
    updateGeometry();
    update();
  }
}

const QPointF & QGraphicsTargetShape::center() const
{
  return center_;
}

void QGraphicsTargetShape::setBaseRadius(double v)
{
  if ( v < 1 ) {
    v = 1;
  }

  if ( baseRadius_ != v ) {
    prepareGeometryChange();
    baseRadius_ = v;
    updateGeometry();
    update();
  }
}

double QGraphicsTargetShape::baseRadius() const
{
  return baseRadius_;
}

void QGraphicsTargetShape::setNumRings(int v)
{
  if ( numRings_ != v ) {
    prepareGeometryChange();
    numRings_ = v;
    updateGeometry();
    update();
  }
}

int QGraphicsTargetShape::numRings() const
{
  return numRings_;
}


void QGraphicsTargetShape::setShowDiagonalRays(bool v)
{
  if ( showDiagonals_ != v ) {
    prepareGeometryChange();
    showDiagonals_ = v;
    updateGeometry();
    update();
  }
}

bool QGraphicsTargetShape::showDiagonalRays() const
{
  return showDiagonals_;
}

void QGraphicsTargetShape::setPen(const QPen & pen)
{
  if ( pen_ != pen ) {
    prepareGeometryChange();
    pen_ = pen;
    updateGeometry();
    update();
  }
}

void QGraphicsTargetShape::setCosmeticPen(const QColor & color, int width )
{
  QPen pen(color);
  pen.setWidth(width);
  pen.setCosmetic(true);
  setPen(pen);
}

void QGraphicsTargetShape::setPenWidth(int v)
{
  if ( pen_.width() != v ) {
    prepareGeometryChange();
    pen_.setWidth(v);
    updateGeometry();
    update();
  }
}

int QGraphicsTargetShape::penWidth() const
{
  return pen_.width();
}

void QGraphicsTargetShape::setPenColor(const QColor & color)
{
  pen_.setColor(color);
  update();
}

QColor QGraphicsTargetShape::penColor() const
{
  return pen_.color();
}

const QPen & QGraphicsTargetShape::pen() const
{
  return pen_;
}

QRectF QGraphicsTargetShape::boundingRect() const
{
  return boundingRect_;
}

QPainterPath QGraphicsTargetShape::shape() const
{
  return shape_;
}

void QGraphicsTargetShape::updateGeometry()
{
  QPainterPath path;

  double r = baseRadius_;
  for( int i = 0; i < numRings_; ++i) {
    path.addEllipse(center_, r, r);
    if( i < numRings_ - 1 ) {
      r *= 2;
    }
  }

  const double ray_length =
      r + baseRadius_;

  path.moveTo(QPointF(center_.x() - ray_length, center_.y()));
  path.lineTo(QPointF(center_.x() + ray_length, center_.y()));


  path.moveTo(QPointF(center_.x(), center_.y() - ray_length));
  path.lineTo(QPointF(center_.x(), center_.y() + ray_length));

  if( showDiagonals_ ) {

    path.moveTo(QPointF(center_.x() - ray_length, center_.y() - ray_length));
    path.lineTo(QPointF(center_.x() + ray_length, center_.y() + ray_length));

    path.moveTo(QPointF(center_.x() - ray_length, center_.y() + ray_length));
    path.lineTo(QPointF(center_.x() + ray_length, center_.y() - ray_length));
  }

  QPen pen = pen_;
  pen.setWidth(15);

  shape_ =
      Base::shapeFromPath(path,
          pen);

  boundingRect_ =
      shape_.boundingRect();
}


void QGraphicsTargetShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  painter->setPen(pen_);

  double r = baseRadius_;
  for( int i = 0; i < numRings_; ++i) {
    painter->drawEllipse(center_, r, r);
    if( i < numRings_ - 1 ) {
      r *= 2;
    }
  }

  const double ray_length =
      r + baseRadius_;

  painter->drawLine(QPointF(center_.x() - ray_length, center_.y()),
      QPointF(center_.x() + ray_length, center_.y()));

  painter->drawLine(QPointF(center_.x(), center_.y() - ray_length),
      QPointF(center_.x(), center_.y() + ray_length));

  if( showDiagonals_ ) {

    painter->drawLine(QPointF(center_.x() - ray_length, center_.y() - ray_length),
        QPointF(center_.x() + ray_length, center_.y() + ray_length));

    painter->drawLine(QPointF(center_.x() - ray_length, center_.y() + ray_length),
        QPointF(center_.x() + ray_length, center_.y() - ray_length));

  }

}

void QGraphicsTargetShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->button() & Qt::LeftButton ) {
    if( fixOnSceneCenter_ ) {
      e->ignore();
      return;
    }
  }
  Base::mousePressEvent(e);
}

void QGraphicsTargetShape::mouseMoveEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->button() & Qt::LeftButton ) {
    if( fixOnSceneCenter_ ) {
      e->ignore();
      return;
    }
  }
  Base::mouseMoveEvent(e);
}

void QGraphicsTargetShape::mouseReleaseEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->button() & Qt::LeftButton ) {
    if( fixOnSceneCenter_ ) {
      e->ignore();
      return;
    }
  }
  Base::mouseReleaseEvent(e);
}

bool QGraphicsTargetShape::fixOnSceneCenter() const
{
  return fixOnSceneCenter_;
}

void QGraphicsTargetShape::setFixOnSceneCenter(bool v)
{
  if( fixOnSceneCenter_ != v ) {

    fixOnSceneCenter_ = v;

    QGraphicsScene *scene =
        this->scene();

    if( scene ) {

      if( !fixOnSceneCenter_ ) {
        scene->disconnect(this);
      }
      else {
        connect(scene, &QGraphicsScene::sceneRectChanged,
            this, &ThisClass::onSceneRectChanged);
      }

      if( fixOnSceneCenter_ ) {
        setCenter(mapFromScene(scene->sceneRect().center()));
      }
    }
  }
}

void QGraphicsTargetShape::onSceneChange()
{
  QGraphicsScene * scene =
      this->scene();

  if ( scene ) {
    scene->disconnect(this);
  }
}

void QGraphicsTargetShape::onSceneHasChanged()
{
  if( fixOnSceneCenter_ ) {

    QGraphicsScene *scene =
        this->scene();

    if( scene ) {
      connect(scene, &QGraphicsScene::sceneRectChanged,
          this, &ThisClass::onSceneRectChanged);
    }
  }
}

void QGraphicsTargetShape::onSceneRectChanged(const QRectF &rect)
{
  if ( fixOnSceneCenter_ ) {
    setCenter(mapFromScene(rect.center()));
  }
}


