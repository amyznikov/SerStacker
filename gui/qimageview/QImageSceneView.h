/*
 * QImageSceneView.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageSceneView_h__
#define __QImageSceneView_h__

#include "QImageScene.h"

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

  void setScale(int scale, const QPoint * centerPos = Q_NULLPTR);
  int scale() const;

  void zoom(int delta);
  void zoom(int delta, QPoint mousePos);
  void resetZoom();

  void setMouseScrollEnabled(bool v);
  bool mouseScrollEnabled() const;

  void scrollView(int dx, int dy);

signals:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void scaleChanged(int currentScale);

protected:
  void wheelEvent(QWheelEvent* e) override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;

protected:
  QImageScene * scene_ = Q_NULLPTR;
  QPoint prevMouseScrollPos_;
  int currentScale_ = 0;
  bool mouseScrollEnabled_ = true;
  bool mouseScrollActive_ = false;

};

#endif /* __QImageSceneView_h__ */
