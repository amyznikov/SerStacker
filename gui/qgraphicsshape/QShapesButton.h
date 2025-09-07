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

class QShapesButton :
    public QToolButton
{
  Q_OBJECT;
public:
  typedef QShapesButton ThisClass;
  typedef QToolButton Base;

  QShapesButton(QWidget * parent = nullptr);
  QShapesButton(QGraphicsView * view, QWidget * parent = nullptr);

  void setSceneView(QGraphicsView * view);
  QGraphicsView * sceneView() const;

  const QList<QGraphicsShape *> & shapes() const;
  void addShape(QGraphicsShape * shape);

protected Q_SLOTS:
  void onPopulateGraphicsShapeContextMenu(QGraphicsShape* shape,
      QMenu & menu, const QPoint & viewpos);

protected:
  QMenu _popup;
  QGraphicsView * _sceneView = nullptr;
  QPen _pen;
  QList<QGraphicsShape *> _shapes;
};

#endif /* __QShapesButton__ */
