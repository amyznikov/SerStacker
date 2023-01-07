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

class QShapesButton
    : public QToolButton
{
  Q_OBJECT;
public:
  typedef QShapesButton ThisClass;
  typedef QToolButton Base;

  QShapesButton(QWidget * parent = Q_NULLPTR);

  void setSceneView(QGraphicsView * view);
  QGraphicsView * sceneView() const;

protected:
  QMenu popup_;
  QGraphicsView * sceneView_ = nullptr;
  QPen pen_;
};

#endif /* __QShapesButton__ */
