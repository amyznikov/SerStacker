/*
 * QGraphicsShapeSettings.cc
 *
 *  Created on: Mar 16, 2024
 *      Author: amyznikov
 */

#include "QGraphicsShape.h"
#include "QGraphicsLineShape.h"
#include "QGraphicsRectShape.h"
#include "QGraphicsTargetShape.h"
// #include <core/debug.h>

QGraphicsShape * QGraphicsShape::load(const QSettings & settings, const QString & sectionName)
{
  const QString shapeType =
      settings.value(QString("%1/type").arg(sectionName)).toString();

  if ( !shapeType.isEmpty() ) {

    if ( shapeType == "line" ) {

      QGraphicsLineShape * line =
          new QGraphicsLineShape();

      line->setLine(settings.value(QString("%1/line").arg(sectionName), line->line()).toLineF());
      line->setArrowSize(settings.value(QString("%1/arrowSize").arg(sectionName), line->arrowSize()).toDouble());
      line->setLockP1(settings.value(QString("%1/lockP1").arg(sectionName), line->lockP1()).toBool());
      line->setLockP2(settings.value(QString("%1/lockP2").arg(sectionName), line->lockP2()).toBool());
      line->setPen(settings.value(QString("%1/pen").arg(sectionName), line->pen()).value<QPen>());
      line->setFlags((QGraphicsItem::GraphicsItemFlags)settings.value(QString("%1/flags").arg(sectionName), (int)line->flags()).value<int>());
      line->setPos(settings.value(QString("%1/pos").arg(sectionName), line->pos()).toPointF());

      return line;
    }

    if ( shapeType == "rect" ) {

      QGraphicsRectShape * rect =
          new QGraphicsRectShape();

      rect->setRect(settings.value(QString("%1/rect").arg(sectionName), rect->rect()).toRectF());
      rect->setPen( settings.value(QString("%1/pen").arg(sectionName), rect->pen()).value<QPen>());
      rect->setBrush(settings.value(QString("%1/brush").arg(sectionName), rect->brush()).value<QBrush>());
      rect->setResizable(settings.value(QString("%1/resizable").arg(sectionName), rect->resizable()).toBool());
      rect->setFixOnSceneCenter(settings.value(QString("%1/fixOnSceneCenter").arg(sectionName), rect->fixOnSceneCenter()).toBool());
      rect->setFlags((QGraphicsItem::GraphicsItemFlags)settings.value(QString("%1/flags").arg(sectionName), (int)rect->flags()).value<int>());
      rect->setPos(settings.value(QString("%1/pos").arg(sectionName), rect->pos()).toPointF());

      return rect;
    }

    if ( shapeType == "target" ) {

      QGraphicsTargetShape * target =
          new QGraphicsTargetShape();

      target->setCenter(settings.value(QString("%1/center").arg(sectionName), target->center()).toPointF());
      target->setBaseRadius(settings.value(QString("%1/baseRadius").arg(sectionName), target->baseRadius()).toDouble());
      target->setNumRings(settings.value(QString("%1/numRings").arg(sectionName), target->numRings()).toInt());
      target->setShowDiagonalRays(settings.value(QString("%1/showDiagonalRays").arg(sectionName), target->showDiagonalRays()).toBool());
      target->setFixOnSceneCenter(settings.value(QString("%1/fixOnSceneCenter").arg(sectionName), target->fixOnSceneCenter()).toBool());
      target->setLockPosition(settings.value(QString("%1/lockPosition").arg(sectionName), target->lockPosition()).toBool());
      target->setPen( settings.value(QString("%1/pen").arg(sectionName), target->pen()).value<QPen>());
      target->setFlags((QGraphicsItem::GraphicsItemFlags)settings.value(QString("%1/flags").arg(sectionName), (int)target->flags()).value<int>());
      target->setPos(settings.value(QString("%1/pos").arg(sectionName), target->pos()).toPointF());

      return target;
    }

  }


  return nullptr;
}

void QGraphicsShape::save(const QGraphicsShape * shape, QSettings & settings, const QString & sectionName)
{
  if ( shape ) {

    if( const QGraphicsLineShape * line = dynamic_cast<const QGraphicsLineShape*>(shape) ) {

      settings.setValue(QString("%1/type").arg(sectionName), QString("line"));
      settings.setValue(QString("%1/line").arg(sectionName), line->line());
      settings.setValue(QString("%1/arrowSize").arg(sectionName), line->arrowSize());
      settings.setValue(QString("%1/lockP1").arg(sectionName), line->lockP1());
      settings.setValue(QString("%1/lockP2").arg(sectionName), line->lockP2());
      settings.setValue(QString("%1/pen").arg(sectionName), line->pen());
      settings.setValue(QString("%1/flags").arg(sectionName), (int)line->flags());
      settings.setValue(QString("%1/pos").arg(sectionName), line->pos());
      // settings.setValue(QString("%1/isVisible").arg(sectionName), line->isVisible());

    }
    else if( const QGraphicsRectShape * rect = dynamic_cast<const QGraphicsRectShape*>(shape) ) {

      settings.setValue(QString("%1/type").arg(sectionName), QString("rect"));
      settings.setValue(QString("%1/rect").arg(sectionName), rect->rect());
      settings.setValue(QString("%1/pen").arg(sectionName), rect->pen());
      settings.setValue(QString("%1/brush").arg(sectionName), rect->brush());
      settings.setValue(QString("%1/resizable").arg(sectionName), rect->resizable());
      settings.setValue(QString("%1/fixOnSceneCenter").arg(sectionName), rect->fixOnSceneCenter());
      settings.setValue(QString("%1/flags").arg(sectionName), (int)rect->flags());
      settings.setValue(QString("%1/pos").arg(sectionName), rect->pos());

    }
    else if( const QGraphicsTargetShape * target = dynamic_cast<const QGraphicsTargetShape*>(shape) ) {

      settings.setValue(QString("%1/type").arg(sectionName), QString("target"));
      settings.setValue(QString("%1/center").arg(sectionName), target->center());
      settings.setValue(QString("%1/baseRadius").arg(sectionName), target->baseRadius());
      settings.setValue(QString("%1/numRings").arg(sectionName), target->numRings());
      settings.setValue(QString("%1/showDiagonalRays").arg(sectionName), target->showDiagonalRays());
      settings.setValue(QString("%1/fixOnSceneCenter").arg(sectionName), target->fixOnSceneCenter());
      settings.setValue(QString("%1/lockPosition").arg(sectionName), target->lockPosition());
      settings.setValue(QString("%1/pen").arg(sectionName), target->pen());
      settings.setValue(QString("%1/flags").arg(sectionName), (int)target->flags());
      settings.setValue(QString("%1/pos").arg(sectionName), target->pos());
    }
    else {
    }

  }

}



