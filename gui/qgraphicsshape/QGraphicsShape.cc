/*
 * QGraphicsShape.cc
 *
 *  Created on: Oct 18, 2021
 *      Author: amyznikov
 */

#include "QGraphicsShape.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<QGraphicsItem::GraphicsItemChange>()
{
  static constexpr c_enum_member members[] = {
      { QGraphicsItem::ItemPositionChange, "ItemPositionChange", "" },
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
      { QGraphicsItem::ItemMatrixChange, "ItemMatrixChange", "" },
#endif
      { QGraphicsItem::ItemVisibleChange, "ItemVisibleChange", "" },
      { QGraphicsItem::ItemEnabledChange, "ItemEnabledChange", "" },
      { QGraphicsItem::ItemSelectedChange, "ItemSelectedChange", "" },
      { QGraphicsItem::ItemParentChange, "ItemParentChange", "" },
      { QGraphicsItem::ItemChildAddedChange, "ItemChildAddedChange", "" },
      { QGraphicsItem::ItemChildRemovedChange, "ItemChildRemovedChange", "" },
      { QGraphicsItem::ItemTransformChange, "ItemTransformChange", "" },
      { QGraphicsItem::ItemPositionHasChanged, "ItemPositionHasChanged", "" },
      { QGraphicsItem::ItemTransformHasChanged, "ItemTransformHasChanged", "" },
      { QGraphicsItem::ItemSceneChange, "ItemSceneChange", "" },
      { QGraphicsItem::ItemVisibleHasChanged, "ItemVisibleHasChanged", "" },
      { QGraphicsItem::ItemEnabledHasChanged, "ItemEnabledHasChanged", "" },
      { QGraphicsItem::ItemSelectedHasChanged, "ItemSelectedHasChanged", "" },
      { QGraphicsItem::ItemParentHasChanged, "ItemParentHasChanged", "" },
      { QGraphicsItem::ItemSceneHasChanged, "ItemSceneHasChanged", "" },
      { QGraphicsItem::ItemCursorChange, "ItemCursorChange", "" },
      { QGraphicsItem::ItemCursorHasChanged, "ItemCursorHasChanged", "" },
      { QGraphicsItem::ItemToolTipChange, "ItemToolTipChange", "" },
      { QGraphicsItem::ItemToolTipHasChanged, "ItemToolTipHasChanged", "" },
      { QGraphicsItem::ItemFlagsChange, "ItemFlagsChange", "" },
      { QGraphicsItem::ItemFlagsHaveChanged, "ItemFlagsHaveChanged", "" },
      { QGraphicsItem::ItemZValueChange, "ItemZValueChange", "" },
      { QGraphicsItem::ItemZValueHasChanged, "ItemZValueHasChanged", "" },
      { QGraphicsItem::ItemOpacityChange, "ItemOpacityChange", "" },
      { QGraphicsItem::ItemOpacityHasChanged, "ItemOpacityHasChanged", "" },
      { QGraphicsItem::ItemScenePositionHasChanged, "ItemScenePositionHasChanged", "" },
      { QGraphicsItem::ItemRotationChange, "ItemRotationChange", "" },
      { QGraphicsItem::ItemRotationHasChanged, "ItemRotationHasChanged", "" },
      { QGraphicsItem::ItemScaleChange, "ItemScaleChange", "" },
      { QGraphicsItem::ItemScaleHasChanged, "ItemScaleHasChanged", "" },
      { QGraphicsItem::ItemTransformOriginPointChange, "ItemTransformOriginPointChange", "" },
      { QGraphicsItem::ItemTransformOriginPointHasChanged, "ItemTransformOriginPointHasChanged", "" },
      { (QGraphicsItem::GraphicsItemChange) (-1) }
  };

  return members;
}

QGraphicsShape::QGraphicsShape(QGraphicsItem *parent) :
    ThisClass("", "", parent)
{
}

QGraphicsShape::QGraphicsShape(const QString & name, const QString & description, QGraphicsItem * parent) :
    Base(parent),
    name_(name),
    description_(description)
{
}

QGraphicsShape::~QGraphicsShape()
{
}

void QGraphicsShape::setName(const QString& name)
{
  name_ = name;
}

const QString & QGraphicsShape::name() const
{
  return name_;
}

void QGraphicsShape::setDescription(const QString& description)
{
  description_ = description;
}

const QString & QGraphicsShape::description() const
{
  return description_;
}

void QGraphicsShape::setRenderHints(QPainter::RenderHints hints, bool on)
{
  if( on ) {
    renderHintsOn_ |= hints;
  }
  else {
    renderHintsOff_ |= hints;
  }
}

QPainter::RenderHints QGraphicsShape::renderHintsOn() const
{
  return renderHintsOn_;
}

QPainter::RenderHints QGraphicsShape::renderHintsOff() const
{
  return renderHintsOff_;
}

void QGraphicsShape::setSnapToPixelGrid(bool v)
{
  snapToPixelGrid_ = v;
}

bool QGraphicsShape::snapToPixelGrid() const
{
  return snapToPixelGrid_;
}

void QGraphicsShape::setUpdatingPos(bool v)
{
  if( v ) {
    ++inUpdatingPos_;
  }
  else if( inUpdatingPos_ && --inUpdatingPos_ < 0 ) {
    inUpdatingPos_ = 0;
  }
}

bool QGraphicsShape::inUpdatingPos() const
{
  return inUpdatingPos_;
}

bool QGraphicsShape::popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  const int n =
      menu.actions().count();

  Q_EMIT populateContextMenuReuested(this, event, &menu);

  return menu.actions().count() != n;
}

void QGraphicsShape::onSceneChange()
{
}

void QGraphicsShape::onSceneHasChanged()
{
}

QVariant QGraphicsShape::itemChange(GraphicsItemChange change, const QVariant & value)
{
//    CF_DEBUG("%s: '%s' : change=%s inUpdatingPos_=%d",
//        this->metaObject()->className(),
//        this->name_.toUtf8().constData(),
//        toString(change),
//        inUpdatingPos_);

  QVariant v =
      Base::itemChange(change, value);

  switch (change) {
    case ItemFlagsChange:
      // ItemSendsGeometryChanges |
      // v.setValue(v.value<quint32>() | ItemSendsScenePositionChanges);
      break;

    case ItemScenePositionHasChanged:
      if( !inUpdatingPos() ) {
        // updateGeo();
      }
      onSceneChange();
      break;

    case ItemSceneChange:
      if( ignoreTransformation_ ) {
        myPreviousScene_ = Base::scene();
      }
      break;

    case ItemSceneHasChanged: {
      if( ignoreTransformation_ ) {

        if( myPreviousScene_ ) {
          setParentItem(ignoreTransformation_->parentItem());
          myPreviousScene_->removeItem(ignoreTransformation_);
        }

        delete ignoreTransformation_;
        ignoreTransformation_ = nullptr;
      }

      QGraphicsScene *newScene =
          Base::scene();

      if( newScene ) {

        if( flags() & QGraphicsItem::ItemIgnoresTransformations ) {

          QGraphicsItem *myParentItem =
              this->parentItem();

          newScene->addItem(ignoreTransformation_ = new QIgnoreTransformationStub());
          setParentItem(ignoreTransformation_);
          ignoreTransformation_->setParentItem(myParentItem);
          ignoreTransformation_->setZValue(this->zValue());
        }
      }

      onSceneHasChanged();
      break;
    }

    case ItemPositionHasChanged: {
      Q_EMIT itemChanged(this);
      break;
    }

    case ItemZValueHasChanged:
      if( ignoreTransformation_ ) {
        ignoreTransformation_->setZValue(this->zValue());
      }
      break;

    default:
      break;
  }

  return v;
}


void QGraphicsShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget )
{
  if( renderHintsOn_ ) {
    painter->setRenderHints(renderHintsOn_, true);
  }
  if( renderHintsOff_ ) {
    painter->setRenderHints(renderHintsOff_, false);
  }
}

void QGraphicsShape::contextMenuEvent(QGraphicsSceneContextMenuEvent * event)
{
  event->ignore();
  Base::contextMenuEvent(event);
  if( event->isAccepted() ) {
    CF_DEBUG("Base::contextMenuEvent() Accepted");
    return;
  }

  QMenu menu;

  if( popuateContextMenu(event, menu) ) {
    menu.addSeparator();
  }

//  QGeoScene *scene =
//      geoScene();
//
//  if( scene && scene->popuateContextMenu(event, menu) ) {
//    menu.addSeparator();
//  }
//
//  QGeoView *view =
//      dynamic_cast<QGeoView*>(getActiveView(event));
//
//  if( view && view->popuateContextMenu(event, menu) ) {
//    menu.addSeparator();
//  }

  if( !menu.isEmpty() ) {
    event->accept();
    menu.exec(event->screenPos());
  }
}

QGraphicsView* QGraphicsShape::getActiveView(const QGraphicsSceneEvent * event)
{
  QWidget *widget = event ? event->widget() : nullptr;
  return widget ? qobject_cast<QGraphicsView*>(widget->parentWidget()) : nullptr;
}

/*!
    copy from source code of qtbase/src/widgets/graphicsview/qgraphicsitem.cpp

    \internal

    Returns a QPainterPath of \a path when stroked with the \a pen.
    Ignoring dash pattern.
*/
QPainterPath QGraphicsShape::shapeFromPath(const QPainterPath &path, const QPen & pen)
{
  // We unfortunately need this hack as QPainterPathStroker will set a width of 1.0
  // if we pass a value of 0.0 to QPainterPathStroker::setWidth()
  const qreal penWidthZero = qreal(0.00000001);
  if( path == QPainterPath() || pen == Qt::NoPen ) {
    return path;
  }
  QPainterPathStroker ps;
  ps.setCapStyle(pen.capStyle());
  if( pen.widthF() <= 0.0 ) {
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

double QGraphicsShape::distance(const QPointF & p1, const QPointF & p2)
{
  return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()));
}

double QGraphicsShape::distance(const QPoint & p1, const QPoint & p2)
{
  return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y()));
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
double QGraphicsShape::distance_from_point_to_line(const QPointF & p, const QPointF & lp1, const QPointF & lp2)
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

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
double QGraphicsShape::distance_from_point_to_line(const QPoint & p, const QPoint & lp1, const QPoint & lp2)
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

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
double QGraphicsShape::distance_from_point_to_line(const QPointF & p, const QLineF & line)
{
  return distance_from_point_to_line(p, line.p1(), line.p2());
}


//
//
/////////////////////////
//
//QGraphicsLineShape::QGraphicsLineShape(const QLineF &line, QGraphicsItem *parent)
//  : Base(line, parent)
//{
//}
//
//QGraphicsLineShape::QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem *parent)
//  : Base(x1, y1, x2, y2, parent)
//{
//}
//
//
//
//
//
//
//
//
//
//void QGraphicsLineShape::mousePressEvent(QGraphicsSceneMouseEvent *e)
//{
//  if ( e->buttons() == Qt::LeftButton ) {
//
//    if ( p1Locked_ && p2Locked_ ) {
//      e->accept();
//      return;
//    }
//
//    const QLineF line =
//        Base::line();
//
//    const QPointF epos =
//        e->pos();
//
//    const QPointF p1 = line.p1();
//    const double p1_distance = hypot(p1.x() - epos.x(), p1.y() - epos.y());
//    if ( p1_distance < hiteps ) {
//      line_moving_mode_ = move_p1;
//      e->accept();
//      return;
//    }
//
//    const QPointF p2 = line.p2();
//    const double p2_distance = hypot(p2.x() - epos.x(), p2.y() - epos.y());
//    if ( p2_distance < hiteps ) {
//      line_moving_mode_ = move_p2;
//      e->accept();
//      return;
//    }
//
//    line_moving_mode_ = move_whole_line;
//  }
//
//  Base::mousePressEvent(e);
//}
//
//void QGraphicsLineShape::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
//{
//  if ( e->buttons() == Qt::LeftButton ) {
//
//    if ( p1Locked_ && p2Locked_ ) {
//      e->accept();
//      return;
//    }
//
//    switch ( line_moving_mode_ ) {
//
//    case move_p1 :
//      if ( p1Locked_ ) {
//        e->accept();
//      }
//      else {
//        QLineF line = Base::line();
//        line.setP1(e->pos());
//        setLine(line);
//        e->accept();
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//      }
//      return;
//
//    case move_p2 :
//      if ( p2Locked_ ) {
//        e->accept();
//      }
//      else {
//        QLineF line = Base::line();
//        line.setP2(e->pos());
//        setLine(line);
//        e->accept();
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//      }
//      return;
//
//    default :
//      if ( p1Locked_ ) {
//        QLineF line = Base::line();
//        const QPointF p1 = line.p1();
//        const QPointF epos = e->pos();
//        const double angle = atan2(epos.y()-p1.y(), epos.x()-p1.x());
//        const QPointF p2(p1.x() + cos(angle) * line.length(), p1.y() + sin(angle) * line.length());
//        line.setP2(p2);
//        setLine(line);
//        e->accept();
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//        return;
//      }
//      if ( p2Locked_ ) {
//        QLineF line = Base::line();
//        const QPointF p2 = line.p2();
//        const QPointF epos = e->pos();
//        const double angle = atan2(epos.y()-p2.y(), epos.x()-p2.x());
//        const QPointF p1(p2.x() + cos(angle) * line.length(), p2.y() + sin(angle) * line.length());
//        line.setP1(p1);
//        setLine(line);
//        e->accept();
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//        return;
//      }
//      line_moving_mode_ = move_whole_line;
//      break;
//    }
//  }
//
//  Base::mouseMoveEvent(e);
//}
//
//void QGraphicsLineShape::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
//{
//  line_moving_mode_ = not_moving;
//  Base::mouseReleaseEvent(event);
//}
//
//QVariant QGraphicsLineShape::itemChange(GraphicsItemChange change, const QVariant &value)
//{
//  if ( change == ItemPositionChange || change == ItemScenePositionHasChanged ) {
//    emit itemChanged(this);
//  }
//  return Base::itemChange(change, value);
//}
//
//QPainterPath QGraphicsLineShape::shape() const
//{
//  const QLineF line =
//      this->line();
//
//  QPainterPath path;
//
//  path.moveTo(line.p1());
//  path.lineTo(line.p2());
//
//  return qt_graphicsItem_shapeFromPath(path, pen());
//}
//
//QRectF QGraphicsLineShape::boundingRect() const
//{
//  return shape().controlPointRect();
//}
//
//void QGraphicsLineShape::populateContextMenu(QMenu & menu, const QGraphicsSceneContextMenuEvent & e)
//{
//  const QPointF epos =
//      mapFromScene(e.scenePos());
//
//  const QLineF line =
//      Base::line();
//
//  const QPointF p1 = line.p1();
//  const QPointF p2 = line.p2();
//
//  if ( hypot(p1.x() - epos.x(), p1.y() - epos.y()) < hiteps ) {
//    if ( p1Locked_ ) {
//      menu.addAction("Unlock p1",
//          [this]() {
//            p1Locked_ = false;
//          });
//    }
//    else {
//      menu.addAction("Lock p1",
//          [this]() {
//            p1Locked_ = true;
//          });
//    }
//  }
//  else if ( hypot(p2.x() - epos.x(), p2.y() - epos.y()) < hiteps ) {
//    if ( p2Locked_ ) {
//      menu.addAction("Unlock p2",
//          [this]() {
//            p2Locked_ = false;
//          });
//    }
//    else {
//      menu.addAction("Lock p2",
//          [this]() {
//            p2Locked_ = true;
//          });
//    }
//  }
//}
//
/////////////////////////////////////////////////////////////////////////////////
//
//QGraphicsRectShape::QGraphicsRectShape(QGraphicsItem *parent)
//  : Base(parent)
//{
//}
//
//QGraphicsRectShape::QGraphicsRectShape(const QRectF &rect, QGraphicsItem *parent)
//  : Base(rect, parent)
//{
//}
//
//QGraphicsRectShape::QGraphicsRectShape(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent)
//  : Base(x, y, w, h, parent)
//{
//}
//
//void QGraphicsRectShape::mousePressEvent(QGraphicsSceneMouseEvent *e)
//{
//  if ( e->buttons() == Qt::LeftButton ) {
//
//    QGraphicsView * view = nullptr;
//    QWidget * widget = e->widget();
//
//    if( widget ) {
//      view = dynamic_cast<QGraphicsView*>(widget);
//      if( !view ) {
//        view = dynamic_cast<QGraphicsView*>(widget->parent());
//      }
//    }
//
//    QPointF epos =
//        e->pos();
//
//    QRectF rect =
//        Base::rect();
//
//    double eps = hiteps;
//
//    if ( view ) {
//      epos = view->mapFromScene(epos);
//      rect.setTopLeft(view->mapFromScene(rect.topLeft()));
//      rect.setBottomRight(view->mapFromScene(rect.bottomRight()));
//
//      if ( rect.width() < 10 || rect.height() < 10 ) {
//        eps = 2;
//      }
//      else if ( rect.width() < 15 || rect.height() < 15 ) {
//        eps = 3;
//      }
//      else if ( rect.width() < 20 || rect.height() < 20 ) {
//        eps = 5;
//      }
//      else {
//        eps = 15;
//      }
//    }
//
//    if (true) {
//
//      QPointF p;
//      int best_corner = not_moving;
//      double distance, best_distance = 1e32;
//
//      p = rect.topLeft();
//      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < eps ) {
//        best_distance = distance;
//        best_corner = move_lt;
//      }
//
//      p = rect.topRight();
//      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < eps ) {
//        if ( distance < best_distance ) {
//          best_distance = distance;
//          best_corner = move_rt;
//        }
//      }
//
//      p = rect.bottomRight();
//      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < eps ) {
//        if ( distance < best_distance ) {
//          best_distance = distance;
//          best_corner = move_rb;
//        }
//      }
//
//      p = rect.bottomLeft();
//      if ( (distance = hypot(p.x() - epos.x(), p.y() - epos.y())) < eps ) {
//        if ( distance < best_distance ) {
//          best_distance = distance;
//          best_corner = move_lb;
//        }
//      }
//
//      if ( best_corner != not_moving ) {
//        rect_moving_mode_ = (rect_moving_mode)best_corner;
//      }
//      else if ( rect.contains(epos) ) {
//        rect_moving_mode_ = move_whole_rect;
//      }
//    }
//  }
//
//  e->accept();
//}
//
//void QGraphicsRectShape::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
//{
//  if ( e->buttons() == Qt::LeftButton ) {
//
//    const QPointF epos =
//        e->pos();
//
//    switch ( rect_moving_mode_ ) {
//    case move_lt : {
//      QRectF rect = Base::rect();
//      if ( epos.x() < rect.bottomRight().x() && epos.y() < rect.bottomRight().y() ) {
//        rect.setTopLeft(epos);
//        setRect(rect);
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//      }
//      e->accept();
//      return;
//    }
//    case move_rt : {
//      QRectF rect = Base::rect();
//      if ( epos.x() > rect.topLeft().x() && epos.y() < rect.bottomRight().y() ) {
//        rect.setTopRight(epos);
//        setRect(rect);
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//      }
//      e->accept();
//      return;
//    }
//    case move_rb : {
//      QRectF rect = Base::rect();
//      if ( epos.x() > rect.topLeft().x() && epos.y() > rect.topLeft().y() ) {
//        rect.setBottomRight(epos);
//        setRect(rect);
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//      }
//      e->accept();
//      return;
//    }
//    case move_lb : {
//      QRectF rect = Base::rect();
//      if ( epos.x() < rect.topRight().x() && epos.y() > rect.topRight().y() ) {
//        rect.setBottomLeft(epos);
//        setRect(rect);
//        if ( flags() & ItemSendsGeometryChanges ) {
//          emit itemChanged(this);
//        }
//      }
//      e->accept();
//      return;
//    }
//    case move_whole_rect: {
//      setPos(pos() + epos - e->lastPos());
//      e->accept();
//      update();
//      return;
//    }
//    default :
//      return;
//    }
//  }
//
//  Base::mouseMoveEvent(e);
//}
//
//void QGraphicsRectShape::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
//{
//  rect_moving_mode_ = not_moving;
//  Base::mouseReleaseEvent(e);
//}
//
//QVariant QGraphicsRectShape::itemChange(GraphicsItemChange change, const QVariant &value)
//{
//  if ( change == ItemPositionChange || change == ItemScenePositionHasChanged ) {
//    emit itemChanged(this);
//  }
//  return Base::itemChange(change, value);
//}
//
//QPainterPath QGraphicsRectShape::shape() const
//{
//  QPainterPath path;
//  path.addRect(rect());
//  return qt_graphicsItem_shapeFromPath(path, pen());
//}
//
//QRectF QGraphicsRectShape::boundingRect() const
//{
//  return shape().controlPointRect();
//}
//


///////////////////////////////////////////////////////////////////////////////
