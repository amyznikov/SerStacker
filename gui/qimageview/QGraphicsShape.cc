/*
 * QGraphicsShape.cc
 *
 *  Created on: Oct 18, 2021
 *      Author: amyznikov
 */

#include "QGraphicsShape.h"
#include "QImageSceneView.h"
#include <core/debug.h>

// hit sensitivity
static const int hiteps = 15;


// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
static double distance_from_point_to_line(const QPointF & p, const QPointF & lp1, const QPointF & lp2)
{
  const QPointF & p1 = lp1;
  const QPointF & p2 = lp2;

  const double x0 = p.x();
  const double y0 = p.y();
  const double x1 = p1.x();
  const double y1 = p1.y();
  const double x2 = p2.x();
  const double y2 = p2.y();

  return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/*! Copied from qtbase source code
    Returns a QPainterPath of \a path when stroked with the \a pen.
    Ignoring dash pattern.
 */
static QPainterPath qt_graphicsItem_shapeFromPath(const QPainterPath &path, const QPen & _pen)
{
  QPen pen = _pen;
  if ( pen.width() < 2 * hiteps ) {
    pen.setWidth(2 * hiteps);
  }

  // We unfortunately need this hack as QPainterPathStroker will set a width of 1.0
  // if we pass a value of 0.0 to QPainterPathStroker::setWidth()
  const qreal penWidthZero = qreal(0.00000001);

  if ( path == QPainterPath() || pen == Qt::NoPen ) {
    return path;
  }
  QPainterPathStroker ps;
  ps.setCapStyle(pen.capStyle());
  if ( pen.widthF() <= 0.0 ) {
    ps.setWidth(penWidthZero);
  }
  else {
    ps.setWidth(pen.widthF());
  }
  ps.setJoinStyle(pen.joinStyle());
  ps.setMiterLimit(pen.miterLimit());
  QPainterPath p = ps.createStroke(path);
  p.addPath(path);
  return p;
}

///////////////////////////////////////////////////////////////////////////////

QGraphicsLineShape::QGraphicsLineShape(QGraphicsItem *parent)
  : Base(parent)
{
}

QGraphicsLineShape::QGraphicsLineShape(const QLineF &line, QGraphicsItem *parent)
  : Base(line, parent)
{
}

QGraphicsLineShape::QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem *parent)
  : Base(x1, y1, x2, y2, parent)
{
}


void QGraphicsLineShape::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
  if ( e->buttons() == Qt::LeftButton ) {

    if ( p1Locked_ && p2Locked_ ) {
      e->accept();
      return;
    }

    const QLineF line =
        Base::line();

    const QPointF epos =
        e->pos();

    const QPointF p1 = line.p1();
    const double p1_distance = hypot(p1.x() - epos.x(), p1.y() - epos.y());
    if ( p1_distance < hiteps ) {
      line_moving_mode_ = move_p1;
      e->accept();
      return;
    }

    const QPointF p2 = line.p2();
    const double p2_distance = hypot(p2.x() - epos.x(), p2.y() - epos.y());
    if ( p2_distance < hiteps ) {
      line_moving_mode_ = move_p2;
      e->accept();
      return;
    }

    line_moving_mode_ = move_whole_line;
  }

  Base::mousePressEvent(e);
}

void QGraphicsLineShape::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
{
  if ( e->buttons() == Qt::LeftButton ) {

    if ( p1Locked_ && p2Locked_ ) {
      e->accept();
      return;
    }

    switch ( line_moving_mode_ ) {

    case move_p1 :
      if ( p1Locked_ ) {
        e->accept();
      }
      else {
        QLineF line = Base::line();
        line.setP1(e->pos());
        setLine(line);
        e->accept();
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
      }
      return;

    case move_p2 :
      if ( p2Locked_ ) {
        e->accept();
      }
      else {
        QLineF line = Base::line();
        line.setP2(e->pos());
        setLine(line);
        e->accept();
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
      }
      return;

    default :
      if ( p1Locked_ ) {
        QLineF line = Base::line();
        const QPointF p1 = line.p1();
        const QPointF epos = e->pos();
        const double angle = atan2(epos.y()-p1.y(), epos.x()-p1.x());
        const QPointF p2(p1.x() + cos(angle) * line.length(), p1.y() + sin(angle) * line.length());
        line.setP2(p2);
        setLine(line);
        e->accept();
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
        return;
      }
      if ( p2Locked_ ) {
        QLineF line = Base::line();
        const QPointF p2 = line.p2();
        const QPointF epos = e->pos();
        const double angle = atan2(epos.y()-p2.y(), epos.x()-p2.x());
        const QPointF p1(p2.x() + cos(angle) * line.length(), p2.y() + sin(angle) * line.length());
        line.setP1(p1);
        setLine(line);
        e->accept();
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
        return;
      }
      line_moving_mode_ = move_whole_line;
      break;
    }
  }

  Base::mouseMoveEvent(e);
}

void QGraphicsLineShape::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
  line_moving_mode_ = not_moving;
  Base::mouseReleaseEvent(event);
}

QVariant QGraphicsLineShape::itemChange(GraphicsItemChange change, const QVariant &value)
{
  if ( change == ItemPositionChange || change == ItemScenePositionHasChanged ) {
    emit onItemChanged(this);
  }
  return Base::itemChange(change, value);
}

QPainterPath QGraphicsLineShape::shape() const
{
  const QLineF line =
      this->line();

  QPainterPath path;

  path.moveTo(line.p1());
  path.lineTo(line.p2());

  return qt_graphicsItem_shapeFromPath(path, pen());
}

QRectF QGraphicsLineShape::boundingRect() const
{
  return shape().controlPointRect();
}

void QGraphicsLineShape::populateContextMenu(QMenu & menu, const QGraphicsSceneContextMenuEvent & e)
{
  const QPointF epos =
      mapFromScene(e.scenePos());

  const QLineF line =
      Base::line();

  const QPointF p1 = line.p1();
  const QPointF p2 = line.p2();

  if ( hypot(p1.x() - epos.x(), p1.y() - epos.y()) < hiteps ) {
    if ( p1Locked_ ) {
      menu.addAction("Unlock p1",
          [this]() {
            p1Locked_ = false;
          });
    }
    else {
      menu.addAction("Lock p1",
          [this]() {
            p1Locked_ = true;
          });
    }
  }
  else if ( hypot(p2.x() - epos.x(), p2.y() - epos.y()) < hiteps ) {
    if ( p2Locked_ ) {
      menu.addAction("Unlock p2",
          [this]() {
            p2Locked_ = false;
          });
    }
    else {
      menu.addAction("Lock p2",
          [this]() {
            p2Locked_ = true;
          });
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

QGraphicsRectShape::QGraphicsRectShape(QGraphicsItem *parent)
  : Base(parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QRectF &rect, QGraphicsItem *parent)
  : Base(rect, parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent)
  : Base(x, y, w, h, parent)
{
}

void QGraphicsRectShape::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
  if ( e->buttons() == Qt::LeftButton ) {

    const QPointF epos =
        e->pos();

    const QRectF rect =
        Base::rect();

    const int margin = 1; //
        std::max(1, pen().width());

    const QRectF rc2 =
        rect.marginsRemoved(QMarginsF(margin, margin, margin, margin));

    CF_DEBUG("rc2.empty=%d rc:(%g %g %g %g) rect:(%g %g %g %g)",
        rc2.isEmpty(),
        rc2.x(), rc2.y(), rc2.width(), rc2.height(),
        rect.x(), rect.y(), rect.width(), rect.height());

    if ( !rc2.isEmpty() && rc2.contains(epos) ) {
      rect_moving_mode_ = move_whole_rect;
      CF_DEBUG("contains");
    }
    else {

      QPointF p;
      int best_corner = move_whole_rect;
      double distance, best_distance = 1e32;

      p = rect.topLeft();
      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < hiteps ) {
        best_distance = distance;
        best_corner = move_lt;
      }

      p = rect.topRight();
      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < hiteps ) {
        if ( distance < best_distance ) {
          best_distance = distance;
          best_corner = move_rt;
        }
      }

      p = rect.bottomRight();
      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < hiteps ) {
        if ( distance < best_distance ) {
          best_distance = distance;
          best_corner = move_rb;
        }
      }

      p = rect.bottomLeft();
      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < hiteps ) {
        if ( distance < best_distance ) {
          best_distance = distance;
          best_corner = move_lb;
        }
      }

      if ( best_corner != move_whole_rect ) {
        rect_moving_mode_ = (rect_moving_mode)best_corner;
        e->accept();
        return;
      }

      if ( rect.contains(epos) ) {
        rect_moving_mode_ = move_whole_rect;
      }
    }
  }

  Base::mousePressEvent(e);
}

void QGraphicsRectShape::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
{
  if ( e->buttons() == Qt::LeftButton ) {

    const QPointF epos =
        e->pos();

    switch ( rect_moving_mode_ ) {
    case move_lt : {
      QRectF rect = Base::rect();
      if ( epos.x() < rect.bottomRight().x() && epos.y() < rect.bottomRight().y() ) {
        rect.setTopLeft(epos);
        setRect(rect);
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
      }
      e->accept();
      return;
    }
    case move_rt : {
      QRectF rect = Base::rect();
      if ( epos.x() > rect.topLeft().x() && epos.y() < rect.bottomRight().y() ) {
        rect.setTopRight(epos);
        setRect(rect);
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
      }
      e->accept();
      return;
    }
    case move_rb : {
      QRectF rect = Base::rect();
      if ( epos.x() > rect.topLeft().x() && epos.y() > rect.topLeft().y() ) {
        rect.setBottomRight(epos);
        setRect(rect);
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
      }
      e->accept();
      return;
    }
    case move_lb : {
      QRectF rect = Base::rect();
      if ( epos.x() < rect.topRight().x() && epos.y() > rect.topRight().y() ) {
        rect.setBottomLeft(epos);
        setRect(rect);
        if ( flags() & ItemSendsGeometryChanges ) {
          emit onItemChanged(this);
        }
      }
      e->accept();
      return;
    }
    default :
      setPos(pos() + epos - e->lastPos());
      e->accept();
      update();
      return;
    }

  }

  Base::mouseMoveEvent(e);
}

void QGraphicsRectShape::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
  rect_moving_mode_ = not_moving;
  Base::mouseReleaseEvent(e);
}

QVariant QGraphicsRectShape::itemChange(GraphicsItemChange change, const QVariant &value)
{
  if ( change == ItemPositionChange || change == ItemScenePositionHasChanged ) {
    emit onItemChanged(this);
  }
  return Base::itemChange(change, value);
}

QPainterPath QGraphicsRectShape::shape() const
{
  QPainterPath path;
  path.addRect(rect());
  return qt_graphicsItem_shapeFromPath(path, pen());
}

QRectF QGraphicsRectShape::boundingRect() const
{
  return shape().controlPointRect();
}



///////////////////////////////////////////////////////////////////////////////
