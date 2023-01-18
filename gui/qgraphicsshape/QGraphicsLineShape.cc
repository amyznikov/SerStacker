/*
 * QGraphicsLineShape.cc
 *
 *  Created on: Jan 16, 2023
 *      Author: amyznikov
 */

#include "QGraphicsLineShape.h"
#include <core/debug.h>

static constexpr int hit_distance = 15;

QGraphicsLineShape::QGraphicsLineShape(QGraphicsItem *parent) :
   Base(parent)
{
}

QGraphicsLineShape::QGraphicsLineShape(const QLineF &line, QGraphicsItem *parent) :
    Base(parent),
    line_(line)
{
}

QGraphicsLineShape::QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem *parent) :
    Base(parent),
    line_(x1, y1, x2, y2)
{
}

QGraphicsLineShape::QGraphicsLineShape(const QPointF & p1, const QPointF & p2, QGraphicsItem * parent) :
    Base(parent),
    line_(p1, p2)
{
}

void QGraphicsLineShape::setLine(const QLineF &line)
{
  line_ = line;
}

void QGraphicsLineShape::setLine(qreal x1, qreal y1, qreal x2, qreal y2)
{
  line_.setLine(x1, y1, x2, y2);
}

const QLineF & QGraphicsLineShape::line() const
{
  return line_;
}

void QGraphicsLineShape::setSceneLine(const QLineF &line)
{
  line_.setPoints(mapFromScene(line.p1()), mapFromScene(line.p1()));
}

void QGraphicsLineShape::setSceneLine(qreal x1, qreal y1, qreal x2, qreal y2)
{
  line_.setPoints(mapFromScene(QPointF(x1, y1)), mapFromScene(QPointF(x2, y2)));
}

QLineF QGraphicsLineShape::sceneLine() const
{
  return QLineF(mapToScene(line_.p1()), mapToScene(line_.p2()));
}

void QGraphicsLineShape::setPen(const QPen & pen)
{
  if ( pen_ != pen ) {
    prepareGeometryChange();
    pen_ = pen;
    updateGeometry();
    update();
  }
}

void QGraphicsLineShape::setCosmeticPen(const QColor & color, int width )
{
  QPen pen(color);
  pen.setWidth(width);
  pen.setCosmetic(true);
  setPen(pen);
}

const QPen & QGraphicsLineShape::pen() const
{
  return pen_;
}

void QGraphicsLineShape::setLockP1(bool v)
{
  lockP1_ = v;
}

bool QGraphicsLineShape::lockP1() const
{
  return lockP1_;
}

void QGraphicsLineShape::setLockP2(bool v)
{
  lockP2_ = v;
}

bool QGraphicsLineShape::lockP2() const
{
  return lockP2_;
}

QRectF QGraphicsLineShape::boundingRect() const
{
  return boundingRect_;
}

QPainterPath QGraphicsLineShape::shape() const
{
  return shape_;
}

void QGraphicsLineShape::updateGeometry()
{
  QPainterPath path;

  path.moveTo(line_.p1());
  path.lineTo(line_.p2());


  QPen pen = pen_;
  pen.setWidth(hit_distance);

  shape_ =
      Base::shapeFromPath(path,
          pen);

  boundingRect_ =
      shape_.boundingRect();
}

void QGraphicsLineShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  painter->setPen(pen_);
  painter->drawLine(line_);
}

void QGraphicsLineShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && (flags() & ItemIsMovable) ) {

    if( lockP1_ && lockP2_ ) {
      e->accept();
      return;
    }

    if( e->modifiers() != Qt::ControlModifier ) {
      e->accept();
      return;
    }

    QGraphicsView *view =
        getActiveView(e);

    if( view ) {

      const QPoint epos =
          view->mapFromScene(e->scenePos());

      const QPoint p1 =
          view->mapFromScene(mapToScene(
              line_.p1()));

      const double p1_distance =
          distance(epos, p1);

      if( p1_distance < hit_distance ) {
        currentMouseAction_ = MouseAction_MoveP1;
        e->accept();
        return;
      }

      const QPoint p2 =
          view->mapFromScene(mapToScene(
              line_.p2()));

      const double p2_distance =
          distance(epos, p2);

      if( p2_distance < hit_distance ) {
        currentMouseAction_ = MouseAction_MoveP2;
        e->accept();
        return;
      }

      currentMouseAction_ = MouseAction_MoveWholeLine;
      e->accept();
      return;
    }

  }

  Base::mousePressEvent(e);
}

void QGraphicsLineShape::mouseMoveEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && (flags() & ItemIsMovable) ) {

    if( lockP1_ && lockP2_ ) {
      e->accept();
      return;
    }

    switch (currentMouseAction_) {

      case MouseAction_MoveP1:
        if( !lockP1_ ) {
          prepareGeometryChange();
          line_.setP1(e->pos());
          updateGeometry();
          update();
        }
        e->accept();
        return;

      case MouseAction_MoveP2:
        if( !lockP2_ ) {
          prepareGeometryChange();
          line_.setP2(e->pos());
          updateGeometry();
          update();
        }
        e->accept();
        return;

      default:
        if( lockP1_ ) {
          prepareGeometryChange();

          const QPointF p1 = line_.p1();
          const QPointF epos = e->pos();

          const double angle =
              atan2(epos.y() - p1.y(), epos.x() - p1.x());

          const QPointF p2(p1.x() + cos(angle) * line_.length(),
              p1.y() + sin(angle) * line_.length());

          line_.setP2(p2);

          updateGeometry();
          update();

          e->accept();
          return;
        }

        if( lockP2_ ) {
          prepareGeometryChange();

          const QPointF p2 = line_.p2();
          const QPointF epos = e->pos();

          const double angle =
              atan2(epos.y() - p2.y(), epos.x() - p2.x());

          const QPointF p1(p2.x() + cos(angle) * line_.length(),
              p2.y() + sin(angle) * line_.length());

          line_.setP1(p1);

          updateGeometry();
          update();

          e->accept();
          return;
        }

        currentMouseAction_ =
            MouseAction_MoveWholeLine;

        Base::mouseMoveEvent(e);
        e->accept();
        return;
    }
  }

  Base::mouseMoveEvent(e);
}

void QGraphicsLineShape::mouseReleaseEvent(QGraphicsSceneMouseEvent * e)
{
  currentMouseAction_ = MouseAction_None;
  Base::mouseReleaseEvent(e);
}

bool QGraphicsLineShape::popuateContextMenu(const QGraphicsSceneContextMenuEvent * e, QMenu & menu)
{
  if ( !lockP1Action_ ) {

    lockP1Action_ = new QAction("Lock P1", this);
    lockP1Action_->setCheckable(true);
    lockP1Action_->setChecked(lockP1_);

    connect(lockP1Action_, &QAction::triggered,
        this, &ThisClass::setLockP1);
  }

  if ( !lockP2Action_ ) {
    lockP2Action_ = new QAction("Lock P2", this);
    lockP2Action_->setCheckable(true);
    lockP2Action_->setChecked(lockP2_);

    connect(lockP2Action_, &QAction::triggered,
        this, &ThisClass::setLockP2);
  }

  menu.addAction(lockP1Action_);
  menu.addAction(lockP2Action_);

  return true;
}

