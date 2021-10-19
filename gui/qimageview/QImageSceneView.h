/*
 * QImageSceneView.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageSceneView_h__
#define __QImageSceneView_h__

#include "QImageScene.h"
#include "QGraphicsShape.h"

class QImageSceneView
    : public QGraphicsView
{
  Q_OBJECT;
public:
  typedef QImageSceneView ThisClass;
  typedef QGraphicsView Base;

  static constexpr int MIN_SCALE = -256;
  static constexpr int MAX_SCALE = +256;


  QImageSceneView(QWidget *parent = Q_NULLPTR);

  QImageScene * scene() const;

  void setViewScale(int scale, const QPoint * centerPos = Q_NULLPTR);
  int viewScale() const;

  void zoom(int delta);
  void zoom(int delta, QPoint mousePos);
  void resetZoom();

  void setMouseScrollEnabled(bool v);
  bool mouseScrollEnabled() const;

  void scrollView(int dx, int dy);

  void setShapesVisible(bool v);
  bool shapesVisible() const;

  void deleteAllShapes();
  void addLineShape();
  void addRectShape();

signals:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void scaleChanged(int currentScale);
  void onLineShapeChanged(QGraphicsLineItem * item);
  void onRectShapeChanged(QGraphicsRectItem * item);

protected:
  void wheelEvent(QWheelEvent* e) override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;

protected:
  QImageScene * scene_ = Q_NULLPTR;
  QPoint prevMouseScrollPos_;
  int currentViewScale_ = 0;
  bool mouseScrollEnabled_ = true;
  bool mouseScrollActive_ = false;
  bool shapesVisible_ = false;
  //QGraphicsItem * currentShape_ = Q_NULLPTR;

//  QGraphicsLineItem * currentLineItem = Q_NULLPTR;
//  int currentLineCorner = -1; // 0:p1, 1:p2, -1:pos
//
//  QGraphicsRectItem * currentRectItem = Q_NULLPTR;
//  int currentRectCorner = -1; // 0:topleft, 1:topright, 2:bottomright, 3:bottomleft, -1: pos

};

#endif /* __QImageSceneView_h__ */
