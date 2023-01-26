/*
 * QShapesButton.h
 *
 *  Created on: Oct 14, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QShapesButton__
#define __QShapesButton__

#include <QtWidgets/QtWidgets>
#include "QGraphicsShape.h"

class QShapesButton
    : public QToolButton
{
  Q_OBJECT;
public:
  typedef QShapesButton ThisClass;
  typedef QToolButton Base;

  QShapesButton(QWidget * parent = nullptr);
  QShapesButton(QGraphicsView * view, QWidget * parent = nullptr);

  void setSceneView(QGraphicsView * view);
  QGraphicsView * sceneView() const;

protected Q_SLOTS:
  void onPopulateGraphicsShapeContextMenu(QGraphicsShape* shape,
      const QGraphicsSceneContextMenuEvent * event,
      QMenu * menu);

protected:
  QMenu popup_;
  QGraphicsView * sceneView_ = nullptr;
  QPen pen_;
  QList<QGraphicsShape *> shapes_;
};

#endif /* __QShapesButton__ */
