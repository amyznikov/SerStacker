/*
 * QGeoPolygonItem.cc
 *
 *  Created on: Nov 27, 2022
 *      Author: amyznikov
 */

#include "QGeoPolygonItem.h"
#include <float.h>
#include <core/debug.h>

namespace {

constexpr int draw_handle_size = 6;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QAbstractGeoPolygonItem::QAbstractGeoPolygonItem(QGraphicsItem * parent) :
  ThisClass("", "", parent)
{
}

QAbstractGeoPolygonItem::QAbstractGeoPolygonItem(const QString & name, const QString & description,
    QGraphicsItem * parent) :
    Base(parent)
{
  pen_.setWidth(1);
  pen_.setCosmetic(true);
  pen_.setColor(QColor(255, 0, 0, 128));

  brush_.setStyle(Qt::BrushStyle::SolidPattern);
  brush_.setColor(QColor(255, 255, 255, 120));
}

QAbstractGeoPolygonItem::~QAbstractGeoPolygonItem()
{
}


void QAbstractGeoPolygonItem::setFillRule(Qt::FillRule v)
{
  fillRule_ = v;
}

Qt::FillRule QAbstractGeoPolygonItem::fillRule() const
{
  return fillRule_;
}


void QAbstractGeoPolygonItem::setEnableMovePoints(bool v)
{
  enableMovePoints_ = v;
}

bool QAbstractGeoPolygonItem::enableMovePoints() const
{
  return enableMovePoints_;
}


void QAbstractGeoPolygonItem::setEnableAddPoints(bool v)
{
  enableAddPoints_ = v;
}

bool QAbstractGeoPolygonItem::enableAddPoints() const
{
  return  enableAddPoints_ ;
}

void QAbstractGeoPolygonItem::setEnableRemovePoints(bool v)
{
  enableRemovePoints_ = v;
}

bool QAbstractGeoPolygonItem::enableRemovePoints() const
{
  return enableRemovePoints_;
}


bool QAbstractGeoPolygonItem::populateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu)
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

  return Base::populateContextMenu(event, menu) || populated;
}

void QAbstractGeoPolygonItem::updateProjected(const QGeoScene * scene)
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

      if ( draw_handle_size > 1 ) {

        for ( int i = 0, n = projPoints_.size(); i < n; ++i ) {

          const QPointF & p =
              projPoints_[i];

          path.addRect(QRectF(p.x() - hit_dstance / 2, p.y() - hit_dstance / 2,
              hit_dstance, hit_dstance));
        }
      }

      projShape_ =
          qt_graphicsItem_shapeFromPath(path, pen_);

      boundingRect_ =
          projShape_.boundingRect();
    }
  }
}

void QAbstractGeoPolygonItem::updateGeo(const QGeoScene * geoScene)
{
  CF_ERROR("FIXME: QGeoMapPolygonItem::updateGeo NOT IMPLEMENTED");
}

QRectF QAbstractGeoPolygonItem::boundingRect() const
{
  return boundingRect_;
}

QPainterPath QAbstractGeoPolygonItem::shape() const
{
  return projShape_;
}

void QAbstractGeoPolygonItem::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  if ( !projPoints_.empty() ) {

    painter->setPen(pen_);
    painter->setBrush(brush_);

    painter->drawPolygon(projPoints_, fillRule_);

    if ( draw_handle_size > 1 ) {

      const QTransform & transform =
          painter->transform();

      const double h =
          draw_handle_size / (std::max)(transform.m11(), transform.m22());

      for ( int i = 0, n = projPoints_.size(); i < n; ++i ) {

        const QPointF & p =
            projPoints_[i];

        painter->fillRect(QRectF(p.x() - h / 2, p.y() - h / 2, h, h),
            pen_.color());
      }
    }
  }
}

void QAbstractGeoPolygonItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
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

void QAbstractGeoPolygonItem::mouseMoveEvent(QGraphicsSceneMouseEvent * event)
{
  //  CF_DEBUG("buttons=0x%0X modifiers=0x%0X",
  //      (uint)event->buttons(),
  //      (uint)event->modifiers());

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

void QAbstractGeoPolygonItem::mouseReleaseEvent(QGraphicsSceneMouseEvent * event)
{
  if ( currentMovingPointIndex_ >= 0 ) {
    currentMovingPointIndex_ = -1;
    event->accept();
    Q_EMIT geoPointsChanged();
    return;
  }

  Base::mouseReleaseEvent(event);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
QGeoPolygonItem::QGeoPolygonItem(QGraphicsItem *parent) :
  ThisClass("", "", parent)
{
}

QGeoPolygonItem::QGeoPolygonItem(const QString & name, const QString & description, QGraphicsItem *parent) :
    Base(parent)
{
  pen_.setWidth(1);
  pen_.setCosmetic(true);
  pen_.setColor(QColor(255, 0, 0, 128));

  brush_.setStyle(Qt::BrushStyle::SolidPattern);
  brush_.setColor(QColor(255, 255, 255, 120));
}


void QGeoPolygonItem::setPoints(const QVector<QGeoPos> & points)
{
//  geoPoints_ = points;
//  updateProjected();
  return setPoints(points.constData(), points.size());
}

void QGeoPolygonItem::setPoints(const std::vector<QGeoPos> & points)
{
  geoPoints_ = points;
  updateProjected();
}

void QGeoPolygonItem::setPoints(const QGeoPos points[], int count)
{
  geoPoints_.clear();
  geoPoints_.reserve(count);
  for( int i = 0; i < count; ++i ) {
    geoPoints_.emplace_back(points[i]);
  }
  updateProjected();
}

int QGeoPolygonItem::pointsCount() const
{
  return geoPoints_.size();
}

void QGeoPolygonItem::insertPoint(const QGeoPos & gp, int insert_pos)
{
  geoPoints_.insert(geoPoints_.begin() + insert_pos, gp);
}

void QGeoPolygonItem::removePoint(int remove_pos)
{
  geoPoints_.erase(geoPoints_.begin() + remove_pos);
}

void QGeoPolygonItem::setGeoPoint(int index, const QGeoPos & gpos)
{
  geoPoints_[index] = gpos;
}

QGeoPos QGeoPolygonItem::getGeoPoint(int index) const
{
  return geoPoints_[index];
}




