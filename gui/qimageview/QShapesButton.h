/*
 * QShapesButton.h
 *
 *  Created on: Oct 14, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QShapesButton__
#define __QShapesButton__

//#include <QtWidgets/QtWidgets>
#include "QImageSceneView.h"

class QShapesButton
    : public QToolButton
{
  Q_OBJECT;
public:
  typedef QShapesButton ThisClass;
  typedef QToolButton Base;

  QShapesButton(QWidget * parent = Q_NULLPTR);

  void setSceneView(QImageSceneView * sceneView);
  QImageSceneView * sceneView() const;

protected:
  QMenu popup_;
  QImageSceneView * sceneView_ = Q_NULLPTR;
};

#endif /* __QShapesButton__ */
