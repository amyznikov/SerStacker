/*
 * QGraphicsTargetShape.cc
 *
 *  Created on: Jan 18, 2023
 *      Author: amyznikov
 */

#include "QGraphicsTargetShape.h"
#include "QGraphicsTargetShapeSettings.h"
#include <core/debug.h>

static const QString myName = "Target circle";
static const QString myDescription = "Target circle shape";

QGraphicsTargetShape::QGraphicsTargetShape(QGraphicsItem * parent) :
  Base(myName, myDescription, parent)
{
  setCosmeticPen(Qt::red, 1);
}

void QGraphicsTargetShape::setCenter(const QPointF & v)
{
  if( _center != v ) {
    prepareGeometryChange();
    _center = v;
    updateGeometry();
    update();
  }
}

const QPointF & QGraphicsTargetShape::center() const
{
  return _center;
}

void QGraphicsTargetShape::setBaseRadius(double v)
{
  if ( v < 1 ) {
    v = 1;
  }

  if ( _baseRadius != v ) {
    prepareGeometryChange();
    _baseRadius = v;
    updateGeometry();
    update();
  }
}

double QGraphicsTargetShape::baseRadius() const
{
  return _baseRadius;
}

void QGraphicsTargetShape::setNumRings(int v)
{
  if ( _numRings != v ) {
    prepareGeometryChange();
    _numRings = v;
    updateGeometry();
    update();
  }
}

int QGraphicsTargetShape::numRings() const
{
  return _numRings;
}


void QGraphicsTargetShape::setShowDiagonalRays(bool v)
{
  if ( _showDiagonals != v ) {
    prepareGeometryChange();
    _showDiagonals = v;
    updateGeometry();
    update();
  }
}

bool QGraphicsTargetShape::showDiagonalRays() const
{
  return _showDiagonals;
}

void QGraphicsTargetShape::setPen(const QPen & pen)
{
  if ( _pen != pen ) {
    prepareGeometryChange();
    _pen = pen;
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
  if ( _pen.width() != v ) {
    prepareGeometryChange();
    _pen.setWidth(v);
    updateGeometry();
    update();
  }
}

int QGraphicsTargetShape::penWidth() const
{
  return _pen.width();
}

void QGraphicsTargetShape::setPenColor(const QColor & color)
{
  _pen.setColor(color);
  update();
}

QColor QGraphicsTargetShape::penColor() const
{
  return _pen.color();
}

const QPen & QGraphicsTargetShape::pen() const
{
  return _pen;
}

QRectF QGraphicsTargetShape::boundingRect() const
{
  return _boundingRect;
}

QPainterPath QGraphicsTargetShape::shape() const
{
  return _shape;
}

void QGraphicsTargetShape::updateGeometry()
{
  QPainterPath path;

  double r = _baseRadius;
  for( int i = 0; i < _numRings; ++i) {
    path.addEllipse(_center, r, r);
    if( i < _numRings - 1 ) {
      r *= 2;
    }
  }

  const double ray_length =
      r + _baseRadius;

  path.moveTo(QPointF(_center.x() - ray_length, _center.y()));
  path.lineTo(QPointF(_center.x() + ray_length, _center.y()));


  path.moveTo(QPointF(_center.x(), _center.y() - ray_length));
  path.lineTo(QPointF(_center.x(), _center.y() + ray_length));

  if( _showDiagonals ) {

    path.moveTo(QPointF(_center.x() - ray_length, _center.y() - ray_length));
    path.lineTo(QPointF(_center.x() + ray_length, _center.y() + ray_length));

    path.moveTo(QPointF(_center.x() - ray_length, _center.y() + ray_length));
    path.lineTo(QPointF(_center.x() + ray_length, _center.y() - ray_length));
  }

  QPen pen = _pen;
  pen.setWidth(15);

  _shape =
      Base::shapeFromPath(path,
          pen);

  _boundingRect =
      _shape.boundingRect();
}


void QGraphicsTargetShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  painter->setPen(_pen);

  double r = _baseRadius;
  for( int i = 0; i < _numRings; ++i) {
    painter->drawEllipse(_center, r, r);
    if( i < _numRings - 1 ) {
      r *= 2;
    }
  }

  const double ray_length =
      r + _baseRadius;

  painter->drawLine(QPointF(_center.x() - ray_length, _center.y()),
      QPointF(_center.x() + ray_length, _center.y()));

  painter->drawLine(QPointF(_center.x(), _center.y() - ray_length),
      QPointF(_center.x(), _center.y() + ray_length));

  if( _showDiagonals ) {

    painter->drawLine(QPointF(_center.x() - ray_length, _center.y() - ray_length),
        QPointF(_center.x() + ray_length, _center.y() + ray_length));

    painter->drawLine(QPointF(_center.x() - ray_length, _center.y() + ray_length),
        QPointF(_center.x() + ray_length, _center.y() - ray_length));

  }

}

void QGraphicsTargetShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->button() & Qt::LeftButton ) {
    if( _fixOnSceneCenter || _lockPosition ) {
      e->ignore();
      return;
    }
  }
  Base::mousePressEvent(e);
}

void QGraphicsTargetShape::mouseMoveEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->button() & Qt::LeftButton ) {
    if( _fixOnSceneCenter || _lockPosition ) {
      e->ignore();
      return;
    }
  }
  Base::mouseMoveEvent(e);
}

void QGraphicsTargetShape::mouseReleaseEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->button() & Qt::LeftButton ) {
    if( _fixOnSceneCenter || _lockPosition ) {
      e->ignore();
      return;
    }
  }
  Base::mouseReleaseEvent(e);
}

bool QGraphicsTargetShape::fixOnSceneCenter() const
{
  return _fixOnSceneCenter;
}

void QGraphicsTargetShape::setFixOnSceneCenter(bool v)
{
  if( _fixOnSceneCenter != v ) {

    _fixOnSceneCenter = v;

    if( QGraphicsScene *scene = this->scene() ) {

      if( !_fixOnSceneCenter ) {
        scene->disconnect(this);
      }
      else {
        connect(scene, &QGraphicsScene::sceneRectChanged,
            this, &ThisClass::onSceneRectChanged);
      }

      if( _fixOnSceneCenter ) {
        setCenter(mapFromScene(scene->sceneRect().center()));
      }
    }
  }
}

bool QGraphicsTargetShape::lockPosition() const
{
  return _lockPosition;
}

void QGraphicsTargetShape::setLockPosition(bool v)
{
  if ( (_lockPosition = v) ) {
  }
}

void QGraphicsTargetShape::onSceneChange()
{
  if ( QGraphicsScene * scene = this->scene() ) {
    scene->disconnect(this);
  }
}

void QGraphicsTargetShape::onSceneHasChanged()
{
  if( _fixOnSceneCenter ) {
    if( QGraphicsScene *scene = this->scene() ) {
      connect(scene, &QGraphicsScene::sceneRectChanged,
          this, &ThisClass::onSceneRectChanged);
    }
  }
}

void QGraphicsTargetShape::onSceneRectChanged(const QRectF &rect)
{
  if ( _fixOnSceneCenter ) {
    setCenter(mapFromScene(rect.center()));
  }
}

void QGraphicsTargetShape::popuateContextMenu(QMenu & menu, const QPoint & viewpos)
{
  QAction * action;

  menu.addSeparator();

  menu.addAction(action = new QAction("Options..."));
  connect(action, &QAction::triggered,
      this, &ThisClass::showShapeSettings);

  menu.addSeparator();

  menu.addAction(action = new QAction("Lock position"));
  action->setCheckable(true);
  action->setChecked(_lockPosition);
  connect(action, &QAction::triggered,
      [this](bool checked) {
        setLockPosition(checked);
      });

  menu.addAction(action = new QAction("Center on scene"));
  action->setCheckable(true);
  action->setChecked(_fixOnSceneCenter);
  connect(action, &QAction::triggered,
      [this](bool checked) {
        setFixOnSceneCenter(checked);
      });

  menu.addSeparator();
  Base::popuateContextMenu(menu, viewpos);
}

void QGraphicsTargetShape::showShapeSettings()
{
  QGraphicsTargetShapeSettingsDialogBox dialogBox("Shape Options",
      this, QApplication::activeWindow());

  dialogBox.exec();
}

