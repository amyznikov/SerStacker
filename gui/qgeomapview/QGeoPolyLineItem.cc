/*
 * QGeoPolyLineItem.cc
 *
 *  Created on: Sep 11, 2024
 *      Author: amyznikov
 */

#include "QGeoPolyLineItem.h"
#include <float.h>
#include <core/debug.h>

namespace {

constexpr int draw_handle_size = 0;
constexpr double hit_dstance = 30;

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

double distance_from_point_to_line(const QPointF & p, const QLineF & line)
{
  return distance_from_point_to_line(p, line.p1(), line.p2());
}

double distance(const QPointF & p1, const QPointF & p2)
{
  return sqrt((p1.x()-p2.x())*(p1.x()-p2.x()) + (p1.y()-p2.y())*(p1.y()-p2.y()));
}

/*!
    copy from source code of qtbase/src/widgets/graphicsview/qgraphicsitem.cpp

    \internal

    Returns a QPainterPath of \a path when stroked with the \a pen.
    Ignoring dash pattern.
*/
QPainterPath qt_graphicsItem_shapeFromPath(const QPainterPath & path, const QPen & pen)
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

}

QAbstractGeoPolyLineItem::QAbstractGeoPolyLineItem(QGraphicsItem * parent) :
  ThisClass("", "", parent)
{
}

QAbstractGeoPolyLineItem::QAbstractGeoPolyLineItem(const QString & name, const QString & description,
    QGraphicsItem * parent) :
    Base(parent)
{
  linePen_.setWidth(3);
  linePen_.setCosmetic(true);
  linePen_.setColor(QColor(0, 255, 128));

  pointPen_.setWidth(5);
  pointPen_.setCosmetic(true);
  pointPen_.setColor(QColor(255, 0, 0, 128));

  //  brush_.setStyle(Qt::BrushStyle::SolidPattern);
  //  brush_.setColor(QColor(255, 255, 255, 120));
}

QAbstractGeoPolyLineItem::~QAbstractGeoPolyLineItem()
{
}


void QAbstractGeoPolyLineItem::setFillRule(Qt::FillRule v)
{
  fillRule_ = v;
}

Qt::FillRule QAbstractGeoPolyLineItem::fillRule() const
{
  return fillRule_;
}

void QAbstractGeoPolyLineItem::setEnableMovePoints(bool v)
{
  enableMovePoints_ = v;
}

bool QAbstractGeoPolyLineItem::enableMovePoints() const
{
  return enableMovePoints_;
}


void QAbstractGeoPolyLineItem::setEnableAddPoints(bool v)
{
  enableAddPoints_ = v;
}

bool QAbstractGeoPolyLineItem::enableAddPoints() const
{
  return  enableAddPoints_ ;
}

void QAbstractGeoPolyLineItem::setEnableRemovePoints(bool v)
{
  enableRemovePoints_ = v;
}

bool QAbstractGeoPolyLineItem::enableRemovePoints() const
{
  return enableRemovePoints_;
}

void QAbstractGeoPolyLineItem::setShowPoints(bool v)
{
  if( showPoints_ != v ) {
    showPoints_ = v;
    updateProjected();
  }
}

bool QAbstractGeoPolyLineItem::showPoints() const
{
  return showPoints_;
}

void QAbstractGeoPolyLineItem::setShowLines(bool v)
{
  if( showLines_ != v ) {
    showLines_ = v;
    updateProjected();
  }
}

bool QAbstractGeoPolyLineItem::showLines() const
{
  return showLines_;
}

void QAbstractGeoPolyLineItem::setPointSize(int v)
{
  if ( pointSize_ != v ) {
    pointSize_ = v;
    updateProjected();
  }
}

int QAbstractGeoPolyLineItem::pointSize() const
{
  return pointSize_;
}

void QAbstractGeoPolyLineItem::setPointPenWidth(int v)
{
  if( pointPen_.width() != v ) {
    pointPen_.setWidth(v);
    updateProjected();
  }
}

int QAbstractGeoPolyLineItem::pointPenWidth() const
{
  return pointPen_.width();
}

void QAbstractGeoPolyLineItem::setPointColor(const QColor & v)
{
  QColor c =
      pointPen_.color();

  if( c.red() != v.red() || c.green() != v.green() || c.blue() != v.blue() ) {
    c.setRed(v.red());
    c.setGreen(v.green());
    c.setBlue(v.blue());
    pointPen_.setColor(c);
    updateProjected();
  }

}

QColor QAbstractGeoPolyLineItem::pointColor() const
{
  return pointPen_.color();
}

void QAbstractGeoPolyLineItem::setPointOpaqueness(int v)
{
  QColor c =
      pointPen_.color();

  if( c.alpha() != v ) {
    c.setAlpha(v);
    pointPen_.setColor(c);
    updateProjected();
  }

}

int QAbstractGeoPolyLineItem::pointOpaqueness() const
{
  return pointPen_.color().alpha();
}

void QAbstractGeoPolyLineItem::setLineWidth(int v)
{
  if ( linePen_.width() != v ) {
    linePen_.setWidth(v);
    updateProjected();
  }
}

int QAbstractGeoPolyLineItem::lineWidth() const
{
  return linePen_.width();
}

void QAbstractGeoPolyLineItem::setLineColor(const QColor & v)
{
  QColor c =
      linePen_.color();

  if( c.red() != v.red() || c.green() != v.green() || c.blue() != v.blue() ) {
    c.setRed(v.red());
    c.setGreen(v.green());
    c.setBlue(v.blue());
    linePen_.setColor(c);
    updateProjected();
  }
}

QColor QAbstractGeoPolyLineItem::lineColor() const
{
  return  linePen_.color();
}

void QAbstractGeoPolyLineItem::setLineOpaqueness(int v)
{
  QColor c =
      linePen_.color();

  if( c.alpha() != v ) {
    c.setAlpha(v);
    linePen_.setColor(c);
    updateProjected();
  }

}

int QAbstractGeoPolyLineItem::lineOpaqueness() const
{
  return linePen_.color().alpha();
}

int QAbstractGeoPolyLineItem::findPointByViewPos(const QGraphicsView * view, const QPointF & viewPos,
    double hit_distance_in_pixels) const
{
  if( view ) {

    const QGeoProjection * projection =
        this->projection();

    const int n =
        pointsCount();

    double best_distance =
        hit_distance_in_pixels;

    int nearest_point_index = -1;

    for( int i = 0; i < n; ++i ) {

      const double dist =
          distance(viewPos,
              view->mapFromScene(mapToScene(projection->geoToProj(getGeoPoint(i)))));

      if( dist < best_distance ) {
        best_distance = dist;
        nearest_point_index = i;
      }
    }

    if( best_distance < hit_distance_in_pixels ) {
      return nearest_point_index;
    }
  }

  return -1;
}

bool QAbstractGeoPolyLineItem::popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
{
  bool populated = false;

  QGraphicsView * view =
      getActiveView(event);

  const QGeoProjection * projection =
      this->projection();

  if ( view ) {

    const QPointF scenePos =
        event->scenePos();

    const QPointF viewPos =
        view->mapFromScene(scenePos);

    const int n =
        pointsCount();

    if( enableAddPoints_ && n > 1 && projection ) {

      double best_distance = DBL_MAX;
      int insertion_pos = -1;

      for( int i = 0; i < n; ++i ) {

        const int i1 = i;
        const int i2 = i < n - 1 ? i + 1 : 0;

        const double distance =
            distance_from_point_to_line(viewPos,
                view->mapFromScene(mapToScene(projection->geoToProj(getGeoPoint(i1)))),
                view->mapFromScene(mapToScene(projection->geoToProj(getGeoPoint(i2)))));

        if ( distance < best_distance ) {
          best_distance = distance;
          insertion_pos = i2;
        }
      }

      if( best_distance < hit_dstance ) {

        if( insertion_pos < 0 || insertion_pos >= n ) {
          CF_ERROR("APP BUG: insertion_pos=%d / [0..%d]",
              insertion_pos, n - 1);
        }

        const QGeoPos geoPos =
            projection->projToGeo(scenePos);

        menu.addAction("Add point",
            [this, insertion_pos, geoPos]() {

              insertPoint(geoPos, insertion_pos);
              updateProjected();

              Q_EMIT geoPointsChanged();
            });

        populated = true;
      }
    }


    if( enableRemovePoints_ && n > 3 && projection ) {

      double best_distance = DBL_MAX;
      int nearest_point_index = -1;

      for( int i = 0; i < n; ++i ) {

        const double dist =
            distance(viewPos,
                view->mapFromScene(mapToScene(projection->geoToProj(getGeoPoint(i)))));

        if ( dist < best_distance ) {
          best_distance = dist;
          nearest_point_index = i;
        }
      }

      if ( best_distance < hit_dstance ) {

        menu.addAction("Remove point",
            [this, nearest_point_index]() {

              removePoint(nearest_point_index);
              updateProjected();

              Q_EMIT geoPointsChanged();
            });
      }

      populated = true;
    }
  }

  return Base::popuateContextMenu(event, menu) || populated;
}

void QAbstractGeoPolyLineItem::updateProjected(const QGeoScene * scene)
{
  if( scene || (scene = geoScene()) ) {

    const QGeoProjection *projection =
        scene->projection();

    if ( projection ) {

      prepareGeometryChange();


      // this->pos() is position of the item in parent coordinates.
      // If the item has no parent, its position is given in scene coordinates.

      const QGraphicsItem * parent =
          parentItem();

      setUpdatingPos(true);
      setPos(0, 0);
      setUpdatingPos(false);

      const int numPoints =
          pointsCount();

      projPoints_.clear();
      projPoints_.resize(numPoints);

      for( int i = 0, n = numPoints; i < n; ++i ) {

        QPointF &pp =
            (projPoints_[i] =
                projection->geoToProj(getGeoPoint(i)));

        if ( parent ) {
          pp -= parent->scenePos();
        }
      }


      QPainterPath path;
      path.addPolygon(projPoints_);

      if ( std::max(pointSize(), lineWidth()) > 1 ) { // draw_handle_size

        for ( int i = 0, n = projPoints_.size(); i < n; ++i ) {

          const QPointF & p =
              projPoints_[i];

          path.addRect(QRectF(p.x() - hit_dstance / 2, p.y() - hit_dstance / 2,
              hit_dstance, hit_dstance));
        }
      }

      projShape_ =
          qt_graphicsItem_shapeFromPath(path, linePen_);

      boundingRect_ =
          projShape_.boundingRect();
    }
  }
}

void QAbstractGeoPolyLineItem::updateGeo(const QGeoScene * geoScene)
{
  CF_ERROR("FIXME: QGeoMapPolygonItem::updateGeo NOT IMPLEMENTED");
}

QRectF QAbstractGeoPolyLineItem::boundingRect() const
{
  return boundingRect_;
}

QPainterPath QAbstractGeoPolyLineItem::shape() const
{
  return projShape_;
}

void QAbstractGeoPolyLineItem::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  if ( !projPoints_.empty() ) {

    //painter->setBrush(brush_);

    if ( showLines_ && linePen_.color().alpha() > 0 ) {
      painter->setPen(linePen_);
      painter->drawPolyline(projPoints_);
    }

    if ( showPoints_ && pointSize_ > 0  && pointPen_.color().alpha() > 0 ) {

      const QTransform & transform =
          painter->transform();

      const double h =
          pointSize_ / (std::max)(transform.m11(), transform.m22());

      painter->setPen(pointPen_);

      for ( int i = 0, n = projPoints_.size(); i < n; ++i ) {

        const QPointF & p =
            projPoints_[i];

//        painter->fillRect(QRectF(p.x() - h / 2, p.y() - h / 2, h, h),
//            pointPen_.color());
        painter->drawRect(QRectF(p.x() - h / 2, p.y() - h / 2, h, h));
      }
    }
  }
}



// Use keyboard modifier like CTRL, ALT or SHIFT in order to receive this event,
// because GeoView uses Mouse move event with left button for view dragging
void QAbstractGeoPolyLineItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
  //  CF_DEBUG("buttons=0x%0X modifiers=0x%0X",
  //      (uint)event->buttons(),
  //      (uint)event->modifiers());

    if( event->buttons() == Qt::LeftButton  && !projPoints_.empty() ) {

      currentMovingPointIndex_ = -1;

      if( enableMovePoints_ ) {

        const QGraphicsView *view =
            getActiveView(event);

        if( view ) {

          double nearest_distance = DBL_MAX;
          int nearest_point = -1;

          const QPoint eventpos =
              view->mapFromScene(event->scenePos());

          for( int i = 0, n = projPoints_.size(); i < n; ++i ) {

            const QPoint viewpos =
                view->mapFromScene(this->mapToScene(
                    projPoints_[i]));

            const double distance =
                sqrt((viewpos.x() - eventpos.x()) * (viewpos.x() - eventpos.x()) +
                    (viewpos.y() - eventpos.y()) * (viewpos.y() - eventpos.y()));

            if( distance < nearest_distance ) {
              nearest_distance = distance;
              nearest_point = i;
            }
          }

          if( nearest_distance < hit_dstance ) {
            currentMovingPointIndex_ = nearest_point;
            event->accept();
            return;
          }
        }
      }
    }

    Base::mousePressEvent(event);
}

void QAbstractGeoPolyLineItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{
    CF_DEBUG("buttons=0x%0X modifiers=0x%0X",
        (uint)event->buttons(),
        (uint)event->modifiers());

    if( currentMovingPointIndex_ >= 0 && (event->buttons() == Qt::LeftButton) ) {

      if( !enableMovePoints_ ) {
        currentMovingPointIndex_ = -1;
      }
      else {

        const QGeoScene *scene =
            geoScene();

        if( scene ) {

          const QGeoProjection *projection =
              scene->projection();

          if( projection ) {

            setGeoPoint(currentMovingPointIndex_,
                projection->projToGeo(event->scenePos()));

            updateProjected(scene);
            event->accept();
            return;
          }
        }
      }
    }

    Base::mouseMoveEvent(event);
}

void QAbstractGeoPolyLineItem::mouseReleaseEvent(QGraphicsSceneMouseEvent * event)
{
  if ( currentMovingPointIndex_ >= 0 ) {
    currentMovingPointIndex_ = -1;
    event->accept();
    Q_EMIT geoPointsChanged();
    return;
  }

  Base::mouseReleaseEvent(event);
}
