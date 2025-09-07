/*
 * QImageSceneView.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageSceneView.h"
#include <gui/qgraphicsshape/QGraphicsShape.h>
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
  return _currentViewScale;
}

void QImageSceneView::setViewScale(int scale, const QPoint * centerPos)
{
  if ( scale < MIN_SCALE ) {
    scale = MIN_SCALE;
  }
  else if ( scale > MAX_SCALE ) {
    scale = MAX_SCALE;
  }

  if ( _currentViewScale != scale ) {

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

    _currentViewScale = scale;
    Q_EMIT scaleChanged(scale);
  }
}

void QImageSceneView::zoom(int delta)
{
  setViewScale(viewScale() + (delta > 0 ? +1 : -1), Q_NULLPTR);
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
  if ( !(_mouseScrollEnabled = v) ) {
    _mouseScrollActive = false;
  }
}

bool QImageSceneView::mouseScrollEnabled() const
{
  return _mouseScrollEnabled;
}

void QImageSceneView::scrollView(int dx, int dy)
{
  horizontalScrollBar()->setValue(horizontalScrollBar()->value() + dx);
  verticalScrollBar()->setValue(verticalScrollBar()->value() + dy);
  Q_EMIT viewScrolled();
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


void QImageSceneView::keyPressEvent(QKeyEvent *e)
{
  if (e->key() == Qt::Key_Menu) {
    // context menu requested
    const QPoint cpos = mapFromGlobal(QCursor::pos());
    if (Base::rect().contains(cpos)) {
      if (handleContextMenuRequest(cpos)) {
        return;
      }
    }
  }

  Base::keyPressEvent(e);
}

//void QImageSceneView::keyReleaseEvent(QKeyEvent *e)
//{
//  Base::keyReleaseEvent(e);
//}

void QImageSceneView::mousePressEvent(QMouseEvent * e)
{
  e->ignore();

  if (e->buttons() == Qt::RightButton && handleContextMenuRequest(e->pos())) {
    e->accept();
    return;
  }

  Q_EMIT onMousePressEvent(e);

  if ( e->isAccepted() ) {
    return;
  }

  if( _mouseScrollEnabled && e->buttons() == Qt::LeftButton && e->modifiers() == Qt::NoModifier ) {
    _mouseScrollActive = true;
    _prevMouseScrollPos = e->pos();
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

  if( _mouseScrollActive && e->buttons() == Qt::LeftButton && e->modifiers() == Qt::NoModifier ) {
    const QPoint mpos = e->pos();
    const QPointF delta = _prevMouseScrollPos - mpos;
    scrollView(delta.x(), delta.y());
    _prevMouseScrollPos = mpos;
    e->accept();
  }
  else if( e->buttons() != Qt::NoButton ) {
    Base::mouseMoveEvent(e);
  }
}

void QImageSceneView::mouseReleaseEvent(QMouseEvent *e)
{
  if( _mouseScrollActive && e->buttons() == Qt::LeftButton && e->modifiers() == Qt::NoModifier ) {
    _mouseScrollActive = false;
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


bool QImageSceneView::handleContextMenuRequest(const QPoint &mpos)
{
  QMenu menu;

  Q_EMIT onPopulateContextMenu(menu, mpos);

  if (!menu.isEmpty()) {
    menu.exec(mapToGlobal(QPoint(mpos.x() - 2, mpos.y() - 2)));
    return true;
  }

  return false;
}

void QImageSceneView::populateContextMenu(QMenu & menu, const QPoint & viewpos)
{
  if (QImageScene *scene = imageScene()) {

    const QPointF scenePos = mapToScene(viewpos);
    QList<QGraphicsItem*> items = scene->items(scenePos);

    if (items.size() > 0) {

      QGraphicsItem *topItem = items.front();

      const QGraphicsItem *groundImageItem = scene->image();
      if (topItem != groundImageItem) {

        const qreal zvalue = topItem->zValue();
        QGraphicsItem *zItem = nullptr;

        for (int i = 1, n = items.size(); i < n; ++i) {
          QGraphicsItem *item = items[i];
          if (item == groundImageItem) {
            break;
          }
          if (item->zValue() >= zvalue) {
            zItem = item;
            break;
          }
        }

        if (QGraphicsShape *shape = dynamic_cast<QGraphicsShape*>(topItem)) {

          const QString objName = shape->name();

          QMenu * subMenu = menu.isEmpty() ? &menu : menu.addMenu(objName.isEmpty() ? "Item" : objName);

          if (zItem) {
            subMenu->addAction("Send to back",
                [this, topItem, zItem]() {
                  topItem->stackBefore(zItem);
                  this->update();
                });
            subMenu->addSeparator();
          }

          shape->popuateContextMenu(*subMenu, viewpos);
        }
        else if (zItem) {
          menu.addAction("Send to back",
              [this, topItem, zItem]() {
                topItem->stackBefore(zItem);
                this->update();
              });

          menu.addSeparator();
        }
      }
    }
  }
}
