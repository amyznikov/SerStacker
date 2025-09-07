/*
 * QImageSceneView.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageSceneView_h__
#define __QImageSceneView_h__

#include "QImageDisplayFunction.h"
#include "QImageScene.h"

class QImageSceneView :
    public QGraphicsView
{
  Q_OBJECT;
public:
  typedef QImageSceneView ThisClass;
  typedef QGraphicsView Base;

  static constexpr int MIN_SCALE = -256;
  static constexpr int MAX_SCALE = +256;

  QImageSceneView(QWidget *parent = nullptr);

  QImageScene * imageScene() const;

  void setViewScale(int scale, const QPoint * centerPos = nullptr);
  int viewScale() const;

  void zoom(int delta);
  void zoom(int delta, QPoint mousePos);
  void resetZoom();

  void setMouseScrollEnabled(bool v);
  bool mouseScrollEnabled() const;

  void scrollView(int dx, int dy);

  virtual void populateContextMenu(QMenu & menu, const QPoint & mpos);

Q_SIGNALS:
  void onMouseMove(QMouseEvent * e);
  void onMousePressEvent(QMouseEvent * e);
  void onMouseReleaseEvent(QMouseEvent * e);
  void onMouseDoubleClick(QMouseEvent * e);
  void onMouseEnterEvent(QEvent *e);
  void onMouseLeaveEvent(QEvent *e);
  void scaleChanged(int currentScale);
  void viewScrolled();
  void onPopulateContextMenu(QMenu & menu, const QPoint & mpos);

protected:
  void keyPressEvent(QKeyEvent *event) override;
  // void keyReleaseEvent(QKeyEvent *event) override;
  void wheelEvent(QWheelEvent* e) override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void leaveEvent(QEvent *event) override;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  void enterEvent(QEnterEvent *event) override;
#else
  void enterEvent(QEvent *event) override;
#endif

protected:
  virtual bool handleContextMenuRequest(const QPoint & mpos);

protected:
  QPoint _prevMouseScrollPos;
  int _currentViewScale = 0;
  bool _mouseScrollEnabled = true;
  bool _mouseScrollActive = false;
};

#endif /* __QImageSceneView_h__ */
