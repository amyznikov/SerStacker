/*
 * QImageSceneView.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageSceneView.h"

QImageSceneView::QImageSceneView(QWidget *parent)
    : Base(parent)
{
  setScene(scene_ = new QImageScene(this));
  setMouseTracking(true);


  QShortcut * shortcut;

  shortcut = new QShortcut(QKeySequence("Ctrl++"), this,
      [this]() {
        zoom(+1);
      }, Qt::WidgetShortcut);

  shortcut = new QShortcut(QKeySequence("Ctrl+-"), this,
      [this]() {
        zoom(-1);
      }, Qt::WidgetShortcut);

  shortcut = new QShortcut(QKeySequence("Ctrl+0"), this,
      this, &ThisClass::resetZoom,
      Qt::WidgetShortcut);

}

QImageScene * QImageSceneView::scene() const
{
  return scene_;
}

int QImageSceneView::scale() const
{
  return currentScale_;
}

void QImageSceneView::setScale(int scale, const QPoint * centerPos)
{
  if ( scale < MIN_SCALE ) {
    scale = MIN_SCALE;
  }
  else if ( scale > MAX_SCALE ) {
    scale = MAX_SCALE;
  }

  if ( currentScale_ != scale ) {

    QPoint mouse_pos = centerPos ? *centerPos : QPoint(width() / 2, height() / 2);
    QPointF old_scene_pos = mapToScene(mouse_pos);

    if ( scale == 0 ) {
      setTransform(QTransform());
    }
    else {
      QTransform t;
      double factor;

      if ( scale < 0 ) {
        factor = 1.0 / (1.0 - 0.2 * scale);
      }
      else {
        factor = (1.0 + 0.2 * scale);
      }

      t.scale(factor, factor);
      setTransform(t);
    }

    const QPointF delta = mapFromScene(old_scene_pos) - mouse_pos;
    horizontalScrollBar()->setValue(delta.x() + horizontalScrollBar()->value());
    verticalScrollBar()->setValue(delta.y() + verticalScrollBar()->value());

    currentScale_ = scale;
    emit scaleChanged(scale);
  }
}

void QImageSceneView::zoom(int delta)
{
  setScale(scale() + (delta > 0 ? +1 : -1), Q_NULLPTR);
}

void QImageSceneView::zoom(int delta, QPoint mousePos)
{
  setScale(scale() + (delta > 0 ? +1 : -1), &mousePos);
}

void QImageSceneView::resetZoom()
{
  setScale(0);
}

void QImageSceneView::setMouseScrollEnabled(bool v)
{
  if ( !(mouseScrollEnabled_ = v) ) {
    mouseScrollActive_ = false;
  }
}

bool QImageSceneView::mouseScrollEnabled() const
{
  return mouseScrollEnabled_;
}

void QImageSceneView::scrollView(int dx, int dy)
{
  horizontalScrollBar()->setValue(horizontalScrollBar()->value() + dx);
  verticalScrollBar()->setValue(verticalScrollBar()->value() + dy);
}



void QImageSceneView::wheelEvent(QWheelEvent* e)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  if ( (e->modifiers() & Qt::ControlModifier) || (e->buttons() == Qt::LeftButton) ) {
    const int amount = e->angleDelta().y();
    if ( amount != 0 ) {
      zoom(amount, QPoint(e->position().x(), e->position().y()));
      return;
    }
  }
  Base::wheelEvent(e);
#else
  if ( (e->modifiers() & Qt::ControlModifier) || (e->buttons() == Qt::LeftButton) ) {
    zoom(e->delta(), e->pos());
  }
  else {
    Base::wheelEvent(e);
  }
#endif
}

void QImageSceneView::mousePressEvent(QMouseEvent * e)
{
  if ( mouseScrollEnabled_ && e->buttons() == Qt::LeftButton ) {

    const QImageScene * scene = this->scene();
    if ( scene ) {
      const QGraphicsItem * item = scene->itemAt(mapToScene(e->pos()), QTransform());
      if ( !item || item == scene->background() ) {
        mouseScrollActive_ = true;
        prevMouseScrollPos_ = e->pos();
        return;
      }
    }
  }

  Base::mousePressEvent(e);
  emit onMousePressEvent(e);
}

void QImageSceneView::mouseMoveEvent(QMouseEvent *e)
{
  if ( !mouseScrollActive_ ) {
    Base::mouseMoveEvent(e);
    emit onMouseMove(e);
  }
  else {
    QPointF delta = prevMouseScrollPos_ - e->pos();
    prevMouseScrollPos_ = e->pos();
    scrollView(delta.x(), delta.y());
  }
}

void QImageSceneView::mouseReleaseEvent(QMouseEvent *e)
{
  if ( mouseScrollActive_ ) {
    mouseScrollActive_ = false;
  }
  else {
    Base::mouseReleaseEvent(e);
    emit onMouseReleaseEvent(e);
  }
}

void QImageSceneView::mouseDoubleClickEvent(QMouseEvent *e)
{
  Base::mouseDoubleClickEvent(e);
  emit onMouseDoubleClick(e);
}

