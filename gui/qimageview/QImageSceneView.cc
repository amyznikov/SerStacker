/*
 * QImageSceneView.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageSceneView.h"
#include <core/debug.h>

QImageSceneView::QImageSceneView(QWidget *parent) :
  Base(parent)
{
  setMouseTracking(true);

  QShortcut * shortcut;

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)

  shortcut =
      new QShortcut(QKeySequence("Ctrl++"), this,
          [this]() {
            zoom(+1);
          }, Qt::WidgetShortcut);

  shortcut =
      new QShortcut(QKeySequence("Ctrl+-"), this,
          [this]() {
            zoom(-1);
          }, Qt::WidgetShortcut);

  shortcut =
      new QShortcut(QKeySequence("Ctrl+0"), this,
          this, &ThisClass::resetZoom,
          Qt::WidgetShortcut);

#else
  shortcut =
      new QShortcut(QKeySequence("Ctrl++"), this,
          nullptr,
          nullptr,
          Qt::WidgetShortcut);

  connect(shortcut, &QShortcut::activated,
      [this]() {
        zoom(+1);
      });

  shortcut =
      new QShortcut(QKeySequence("Ctrl+-"), this,
          nullptr,
          nullptr,
          Qt::WidgetShortcut);

  connect(shortcut, &QShortcut::activated,
      [this]() {
        zoom(-1);
      });

  shortcut =
      new QShortcut(QKeySequence("Ctrl+0"), this,
          nullptr,
          nullptr,
          Qt::WidgetShortcut);

  connect(shortcut, &QShortcut::activated,
      this, &ThisClass::resetZoom);

#endif

}

QImageScene * QImageSceneView::imageScene() const
{
  return dynamic_cast<QImageScene *>(Base::scene());
}

int QImageSceneView::viewScale() const
{
  return currentViewScale_;
}

void QImageSceneView::setViewScale(int scale, const QPoint * centerPos)
{
  if ( scale < MIN_SCALE ) {
    scale = MIN_SCALE;
  }
  else if ( scale > MAX_SCALE ) {
    scale = MAX_SCALE;
  }

  if ( currentViewScale_ != scale ) {

    QPoint mouse_pos =
        centerPos ? *centerPos :
            QPoint(width() / 2, height() / 2);

    QPointF old_scene_pos =
        mapToScene(mouse_pos);

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

    const QPointF delta =
        mapFromScene(old_scene_pos) - mouse_pos;

    horizontalScrollBar()->setValue(delta.x() + horizontalScrollBar()->value());
    verticalScrollBar()->setValue(delta.y() + verticalScrollBar()->value());

    currentViewScale_ = scale;
    Q_EMIT scaleChanged(scale);
  }
}

void QImageSceneView::zoom(int delta)
{
  setViewScale(viewScale() + (delta > 0 ? +1 : -1), nullptr);
}

void QImageSceneView::zoom(int delta, QPoint mousePos)
{
  setViewScale(viewScale() + (delta > 0 ? +1 : -1), &mousePos);
}

void QImageSceneView::resetZoom()
{
  setViewScale(0);
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
  Q_EMIT viewScrolled();
}

void QImageSceneView::wheelEvent(QWheelEvent* e)
{
 // CF_DEBUG("Scene: %p", scene_);

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
  e->ignore();

  Q_EMIT onMousePressEvent(e);

  if ( e->isAccepted() ) {
    return;
  }

  if( mouseScrollEnabled_ && e->buttons() == Qt::LeftButton && e->modifiers() == Qt::NoModifier ) {
    mouseScrollActive_ = true;
    prevMouseScrollPos_ = e->pos();
    e->accept();
  }
  else {
    Base::mousePressEvent(e);
  }
}

void QImageSceneView::mouseMoveEvent(QMouseEvent * e)
{
  e->ignore();

  Q_EMIT onMouseMove(e);
  if( e->isAccepted() ) {
    return;
  }

  if( mouseScrollActive_ && e->buttons() == Qt::LeftButton && e->modifiers() == Qt::NoModifier ) {
    const QPoint mpos = e->pos();
    const QPointF delta = prevMouseScrollPos_ - mpos;
    scrollView(delta.x(), delta.y());
    prevMouseScrollPos_ = mpos;
    e->accept();
  }
  else if( e->buttons() != Qt::NoButton ) {
    Base::mouseMoveEvent(e);
  }
}

void QImageSceneView::mouseReleaseEvent(QMouseEvent *e)
{
  if( mouseScrollActive_ && e->buttons() == Qt::LeftButton && e->modifiers() == Qt::NoModifier ) {
    mouseScrollActive_ = false;
  }
  else {
    Base::mouseReleaseEvent(e);
  }

  Q_EMIT onMouseReleaseEvent(e);
}

void QImageSceneView::mouseDoubleClickEvent(QMouseEvent *e)
{
  Base::mouseDoubleClickEvent(e);
  Q_EMIT onMouseDoubleClick(e);
}

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
void QImageSceneView::enterEvent(QEnterEvent *e)
{
  Base::enterEvent(e);
  Q_EMIT onMouseEnterEvent(e);
}
#else
void QImageSceneView::enterEvent(QEvent *e)
{
  Base::enterEvent(e);
  Q_EMIT onMouseEnterEvent(e);
}
#endif

void QImageSceneView::leaveEvent(QEvent *e)
{
  Base::leaveEvent(e);
  Q_EMIT onMouseLeaveEvent(e);
}

