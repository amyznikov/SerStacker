/*
 * QGraphicsRectShape.cc
 *
 *  Created on: Jan 4, 2023
 *      Author: amyznikov
 */

#include "QGraphicsRectShape.h"
#include <float.h>
#include <core/debug.h>

namespace {

double distance(const QPointF & p1, const QPointF & p2)
{
  return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()));
}

double distance(const QPoint & p1, const QPoint & p2)
{
  return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()));
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
double distance_from_point_to_line(const QPointF & p, const QPointF & lp1, const QPointF & lp2)
{
  const QPointF &p1 = lp1;
  const QPointF &p2 = lp2;

  const double x0 = p.x();
  const double y0 = p.y();
  const double x1 = p1.x();
  const double y1 = p1.y();
  const double x2 = p2.x();
  const double y2 = p2.y();

  return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double distance_from_point_to_line(const QPoint & p, const QPoint & lp1, const QPoint & lp2)
{
  const QPoint &p1 = lp1;
  const QPoint &p2 = lp2;

  const double x0 = p.x();
  const double y0 = p.y();
  const double x1 = p1.x();
  const double y1 = p1.y();
  const double x2 = p2.x();
  const double y2 = p2.y();

  return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double distance_from_point_to_line(const QPointF & p, const QLineF & line)
{
  return distance_from_point_to_line(p, line.p1(), line.p2());
}


} //

QGraphicsRectShape::QGraphicsRectShape(QGraphicsItem * parent) :
    Base(parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QRectF & rect, QGraphicsItem * parent) :
    Base(parent),
    rect_(rect)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QString & name, const QString & description, QGraphicsItem * parent) :
    Base(name, description, parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QString & name, const QString & description, const QRectF & rect,
    QGraphicsItem * parent) :
    Base(name, description, parent),
    rect_(rect)
{
}

void QGraphicsRectShape::setRect(const QRectF & rc)
{
  if (rect_ != rc ) {
    prepareGeometryChange();
    rect_ = rc;
    updateGeometry();
    update();
  }
}

const QRectF & QGraphicsRectShape::rect() const
{
  return rect_;
}

QRectF QGraphicsRectShape::sceneRect() const
{
  return mapToScene(rect_).boundingRect();
}


void QGraphicsRectShape::setPen(const QPen & pen)
{
  if ( pen_ != pen ) {
    prepareGeometryChange();
    pen_ = pen;
    updateGeometry();
    update();
  }
}

void QGraphicsRectShape::setCosmeticPen(const QColor & color, int width )
{
  QPen pen(color);
  pen.setWidth(width);
  pen.setCosmetic(true);
  setPen(pen);
}


const QPen & QGraphicsRectShape::pen() const
{
  return pen_;
}

void QGraphicsRectShape::setBrush(const QBrush & brush)
{
  if ( brush_ != brush ) {
    prepareGeometryChange();
    brush_ = brush;
    // updateGeometry();
    update();
  }
}

const QBrush & QGraphicsRectShape::brush() const
{
  return brush_;
}

void QGraphicsRectShape::setResizable(bool v)
{
  itemIsResizable_ = v;
}

bool QGraphicsRectShape::resizable() const
{
  return itemIsResizable_;
}

void QGraphicsRectShape::updateGeometry()
{
  QPainterPath path;

  boundingRect_ =
      rect_.adjusted(-hitDstance_, -hitDstance_,
          +hitDstance_, +hitDstance_);


  path.addRect(boundingRect_);

  shape_ =
      Base::shapeFromPath(path, pen());
}

QRectF QGraphicsRectShape::boundingRect() const
{
  return boundingRect_;
}

QPainterPath QGraphicsRectShape::shape() const
{
  return shape_;
}

void QGraphicsRectShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  if( !rect_.isEmpty() ) {

    painter->setPen(pen_);
    painter->setBrush(brush_);

    painter->drawRect(rect_);
  }
}

void QGraphicsRectShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {

    currentMouseAction_  = MouseAction_None;

    if( itemIsResizable_ && e->modifiers() == Qt::ControlModifier) {

      QGraphicsView *view =
          getActiveView(e);

      if( view ) {

        const QPoint viewpos =
            view->mapFromScene(e->scenePos());

        const QPoint corners[4] = {
            view->mapFromScene(mapToScene(rect_.topLeft())),
            view->mapFromScene(mapToScene(rect_.topRight())),
            view->mapFromScene(mapToScene(rect_.bottomRight())),
            view->mapFromScene(mapToScene(rect_.bottomLeft())),
        };

        // Try corners first

        double best_corner_distance = DBL_MAX;
        int best_corner = -1;

        for( int i = 0; i < 4; ++i ) {

          const double d =
              distance(viewpos,
                  corners[i]);

          if( d < hitDstance_ && d < best_corner_distance ) {
            best_corner_distance = d;
            best_corner = i;
          }
        }

        if( best_corner != -1 ) {

          switch (best_corner) {
            case 0:
              currentMouseAction_ = MouseAction_MoveTopLeft;
              break;
            case 1:
              currentMouseAction_ = MouseAction_MoveTopRight;
              break;
            case 2:
              currentMouseAction_ = MouseAction_MoveBottomRight;
              break;
            case 3:
              currentMouseAction_ = MouseAction_MoveBottomLeft;
              break;
          }

          e->accept();
          return;
        }

        // Try side borders now

        double best_side_distance = DBL_MAX;
        int best_side = -1;

        for( int i1 = 0; i1 < 4; ++i1 ) {

          const int i2 =
              i1 < 3 ? i1 + 1 :
                  0;

          const double d =
              distance_from_point_to_line(viewpos,
                  corners[i1], corners[i2]);

          if( d < hitDstance_ && d < best_side_distance ) {
            best_side_distance = d;
            best_side = i1;
          }
        }

        if( best_side != -1 ) {

          switch (best_side) {
            case 0:
              currentMouseAction_ = MouseAction_MoveTop;
              break;
            case 1:
              currentMouseAction_ = MouseAction_MoveRight;
              break;
            case 2:
              currentMouseAction_ = MouseAction_MoveBottom;
              break;
            case 3:
              currentMouseAction_ = MouseAction_MoveLeft;
              break;
          }

          e->accept();
          return;
        }
      }
    }
  }

  // Try move
  if( (flags() & ItemIsMovable) && (e->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier)) ) {
    Base::mousePressEvent(e);
    e->accept();
    return;
  }

  e->ignore();
}

void QGraphicsRectShape::mouseMoveEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {

    if( (flags() & ItemIsMovable) && e->modifiers() == Qt::ShiftModifier ) {
      Base::mouseMoveEvent(e);
      e->accept();
      return;
    }

    if( e->modifiers() == Qt::ControlModifier ) {

      const QPointF pos = e->pos();

      bool hasChanges = true;

      switch (currentMouseAction_) {
        case MouseAction_MoveTop:
          if( pos.y() <= rect_.bottom() - 1 ) {
            rect_.setTop(pos.y());
          }
          break;
        case MouseAction_MoveRight:
          if( pos.x() >= rect_.left() + 1 ) {
            rect_.setRight(pos.x());
          }
          break;
        case MouseAction_MoveBottom:
          if( pos.y() >= rect_.top() + 1 ) {
            rect_.setBottom(pos.y());
          }
          break;
        case MouseAction_MoveLeft:
          if( pos.x() <= rect_.right() - 1 ) {
            rect_.setLeft(pos.x());
          }
          break;

        case MouseAction_MoveTopLeft:
          if( pos.y() <= rect_.bottom() - 1 ) {
            rect_.setTop(pos.y());
          }
          if( pos.x() <= rect_.right() - 1 ) {
            rect_.setLeft(pos.x());
          }
          break;
        case MouseAction_MoveTopRight:
          if( pos.y() <= rect_.bottom() - 1 ) {
            rect_.setTop(pos.y());
          }
          if( pos.x() >= rect_.left() + 1 ) {
            rect_.setRight(pos.x());
          }
          break;
        case MouseAction_MoveBottomRight:
          if( pos.y() >= rect_.top() + 1 ) {
            rect_.setBottom(pos.y());
          }
          if( pos.x() >= rect_.left() + 1 ) {
            rect_.setRight(pos.x());
          }
          break;
        case MouseAction_MoveBottomLeft:
          if( pos.y() >= rect_.top() + 1 ) {
            rect_.setBottom(pos.y());
          }
          if( pos.x() <= rect_.right() - 1 ) {
            rect_.setLeft(pos.x());
          }
          break;
        default:
          hasChanges = false;
          break;
      }

      if( hasChanges ) {
        prepareGeometryChange();
        updateGeometry();
        update();

        if( (flags() & (ItemSendsGeometryChanges | ItemSendsScenePositionChanges)) ) {
          Q_EMIT itemChanged(this);
        }
      }
      else if( (flags() & ItemIsMovable) ) {
        Base::mouseMoveEvent(e);
      }

      e->accept();
      return;
    }
  }

  Base::mouseMoveEvent(e);
}

void QGraphicsRectShape::mouseReleaseEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {
    currentMouseAction_ = MouseAction_None;
  }
  Base::mouseReleaseEvent(e);
}
