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
  static const c_enum_member members[] = {
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
  if( (snapToPixelGrid_ != v) && (snapToPixelGrid_ = v) ) {
    prepareGeometryChange();
    updateGeometry();
    update();
  }
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

void QGraphicsShape::popuateContextMenu(QMenu & menu, const QPoint & viewpos)
{
  Q_EMIT populateContextMenuReuested(this, menu, viewpos);
}

void QGraphicsShape::onSceneChange()
{
}

void QGraphicsShape::onSceneHasChanged()
{
}

void QGraphicsShape::updateGeometry()
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

///////////////////////////////////////////////////////////////////////////////
