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
#include "QGraphicsShapeSettings.h"
// #include <core/debug.h>

void QGraphicsShape::load(QGraphicsShape * shape, const QSettings & settings, const QString & sectionName)
{
  if( shape ) {
    shape->loadSettings(settings, sectionName);
  }
}

void QGraphicsShape::save(const QGraphicsShape * shape, QSettings & settings, const QString & sectionName)
{
  if( shape ) {
    if( dynamic_cast<const QGraphicsLineShape*>(shape) ) {
      settings.setValue(QString("%1/type").arg(sectionName), QString("line"));
    }
    else if( dynamic_cast<const QGraphicsRectShape*>(shape) ) {
      settings.setValue(QString("%1/type").arg(sectionName), QString("rect"));
    }
    else if( const QGraphicsTargetShape * target = dynamic_cast<const QGraphicsTargetShape*>(shape) ) {
      settings.setValue(QString("%1/type").arg(sectionName), QString("target"));
    }
    else {
    }

    shape->saveSettings(settings, sectionName);
  }
}

QGraphicsShape* QGraphicsShape::load(const QSettings & settings, const QString & sectionName)
{
  const QString shapeType =
      settings.value(QString("%1/type").arg(sectionName)).toString();

  if( !shapeType.isEmpty() ) {

    if( shapeType == "line" ) {
      QGraphicsLineShape * obj = new QGraphicsLineShape();
      load(obj, settings, sectionName);
      return obj;
    }

    if( shapeType == "rect" ) {
      QGraphicsRectShape * obj = new QGraphicsRectShape();
      load(obj, settings, sectionName);

      return obj;
    }

    if( shapeType == "target" ) {
      QGraphicsTargetShape * obj = new QGraphicsTargetShape();
      load(obj, settings, sectionName);
      return obj;
    }
  }

  return nullptr;
}

