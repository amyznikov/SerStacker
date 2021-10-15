/*
 * QImageSceneView.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageSceneView.h"
#include <core/debug.h>

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
//static double distance_from_point_to_line(const QPointF & p, const QLineF & l)
//{
//  const QPointF p1 = l.p1();
//  const QPointF p2 = l.p2();
//
//  const double x0 = p.x();
//  const double y0 = p.y();
//  const double x1 = p1.x();
//  const double y1 = p1.y();
//  const double x2 = p2.x();
//  const double y2 = p2.y();
//
//  return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
//}

static double distance_from_point_to_line(const QPointF & p, const QPointF & lp1, const QPointF & lp2)
{
  const QPointF & p1 = lp1;
  const QPointF p2 = lp2;

  const double x0 = p.x();
  const double y0 = p.y();
  const double x1 = p1.x();
  const double y1 = p1.y();
  const double x2 = p2.x();
  const double y2 = p2.y();

  return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

QImageSceneView::QImageSceneView(QWidget *parent)
    : Base(parent)
{
  setScene(scene_ = new QImageScene(this));
  scene_->setBackgroundBrush(QBrush(Qt::darkGray, Qt::SolidPattern));

  setMouseTracking(true);

  QShortcut * shortcut;

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)

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

QImageScene * QImageSceneView::scene() const
{
  return scene_;
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

    currentViewScale_ = scale;
    emit scaleChanged(scale);
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
  bool handled = false;

  currentLineItem = Q_NULLPTR;
  currentRectItem = Q_NULLPTR;

  if ( e->buttons() == Qt::LeftButton ) {

    const QPoint mpos =
      e->pos();

    const QImageScene * scene =
        this->scene();

    const QList<QGraphicsItem *> items =
        scene->items();

    QGraphicsItem * bgitem =
        scene->background();


    QGraphicsLineItem * lineItem;
    QGraphicsRectItem * rectItem;

    static constexpr int hiteps = 15;

    for ( QGraphicsItem * item : items ) {
      if ( item != bgitem ) {

        if ( (lineItem = dynamic_cast<QGraphicsLineItem*>(item)) ) {

          QLineF line =
              lineItem->line();

          const QPointF p1 = mapFromScene(lineItem->pos() + line.p1());
          if ( hypot(p1.x() - mpos.x(), p1.y() - mpos.y()) < hiteps ) {
            currentLineItem = lineItem;
            currentLineCorner = 0;
            handled = true;
            break;
          }

          const QPointF p2 = mapFromScene(lineItem->pos() + line.p2());
          if ( hypot(p2.x() - mpos.x(), p2.y() - mpos.y()) < hiteps ) {
            currentLineItem = lineItem;
            currentLineCorner = 1;
            handled = true;
            break;
          }

          if ( distance_from_point_to_line(mpos, p1, p2) < hiteps ) {
            currentLineItem = lineItem;
            currentLineCorner = -1;
            handled = true;
            break;
          }

        }

        else if ( (rectItem = dynamic_cast<QGraphicsRectItem*>(item)) ) {

          QPointF p;

          QRectF rect =
              rectItem->rect();

          p = mapFromScene(rectItem->pos() + rect.topLeft());
          if ( hypot(p.x() - mpos.x(), p.y() - mpos.y()) < hiteps ) {
            currentRectItem = rectItem;
            currentRectCorner = 0;
            handled = true;
            break;
          }

          p = mapFromScene(rectItem->pos() + rect.topRight());
          if ( hypot(p.x() - mpos.x(), p.y() - mpos.y()) < hiteps ) {
            currentRectItem = rectItem;
            currentRectCorner = 1;
            handled = true;
            break;
          }

          p = mapFromScene(rectItem->pos() + rect.bottomRight());
          if ( hypot(p.x() - mpos.x(), p.y() - mpos.y()) < hiteps ) {
            currentRectItem = rectItem;
            currentRectCorner = 2;
            handled = true;
            break;
          }

          p = mapFromScene(rectItem->pos() + rect.bottomLeft());
          if ( hypot(p.x() - mpos.x(), p.y() - mpos.y()) < hiteps ) {
            currentRectItem = rectItem;
            currentRectCorner = 3;
            handled = true;
            break;
          }

          if ( rect.translated(rectItem->pos()).contains(mapToScene(mpos)) ) {
            currentRectItem = rectItem;
            currentRectCorner = -1;
            handled = true;
            break;
          }
        }
      }
    }

    if ( handled ) {
      prevMouseScrollPos_ = mpos;
    }
    else if ( mouseScrollEnabled_ ) {

      const QImageScene * scene = this->scene();
      if ( scene ) {
        const QGraphicsItem * item = scene->itemAt(mapToScene(e->pos()), QTransform());
        if ( !item || item == scene->background() ) {
          mouseScrollActive_ = true;
          prevMouseScrollPos_ = mpos;
          return;
        }
      }
    }
  }

  if ( !handled ) {
    Base::mousePressEvent(e);
    emit onMousePressEvent(e);
  }
}

void QImageSceneView::mouseMoveEvent(QMouseEvent *e)
{
  if ( mouseScrollActive_ ) {
    const QPoint mpos = e->pos();
    const QPointF delta = prevMouseScrollPos_ - mpos;
    scrollView(delta.x(), delta.y());
    prevMouseScrollPos_ = mpos;
  }
  else if ( currentLineItem ) {

    const QPoint mpos =
        e->pos();

    const QPointF spos =
        mapToScene(mpos);

    switch ( currentLineCorner ) {
    case 0 : {
      QLineF line = currentLineItem->line();
      line.setP1(spos - currentLineItem->pos());
      currentLineItem->setLine(line);
      break;
    }
    case 1 : {
      QLineF line = currentLineItem->line();
      line.setP2(spos - currentLineItem->pos());
      currentLineItem->setLine(line);
      break;
    }
    default :
      currentLineItem->setPos(currentLineItem->pos() +
          spos - mapToScene(prevMouseScrollPos_));
      break;
    }

    prevMouseScrollPos_ = mpos;

    emit onLineShapeChanged(currentLineItem);
  }
  else if ( currentRectItem ) {

    const QPoint mpos =
        e->pos();

    const QPointF spos =
        mapToScene(mpos);

    switch ( currentRectCorner ) {
    case 0 : {
      QRectF rect = currentRectItem->rect();
      rect.setTopLeft(spos - currentRectItem->pos());
      currentRectItem->setRect(rect);
      break;
    }
    case 1 : {
      QRectF rect = currentRectItem->rect();
      rect.setTopRight(spos - currentRectItem->pos());
      currentRectItem->setRect(rect);
      break;
    }
    case 2 : {
      QRectF rect = currentRectItem->rect();
      rect.setBottomRight(spos - currentRectItem->pos());
      currentRectItem->setRect(rect);
      break;
    }
    case 3 : {
      QRectF rect = currentRectItem->rect();
      rect.setBottomLeft(spos - currentRectItem->pos());
      currentRectItem->setRect(rect);
      break;
    }
    default :
      currentRectItem->setPos(currentRectItem->pos() +
          spos - mapToScene(prevMouseScrollPos_));
      break;
    }

    prevMouseScrollPos_ = mpos;
    emit onRectShapeChanged(currentRectItem);
  }
  else {
    Base::mouseMoveEvent(e);
    emit onMouseMove(e);
  }
}

void QImageSceneView::mouseReleaseEvent(QMouseEvent *e)
{
  if ( mouseScrollActive_ ) {
    mouseScrollActive_ = false;
  }
  else if ( currentLineItem ) {
    currentLineItem = Q_NULLPTR;
  }
  else if ( currentRectItem ) {
    currentRectItem = Q_NULLPTR;
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


void QImageSceneView::setShapesVisible(bool v)
{
  //if ( shapesVisible_ != v )
  {
    shapesVisible_ = v;

    if ( scene_ ) {

      const QList<QGraphicsItem *> items = scene_->items();
      const QGraphicsPixmapItem * bgitem = scene_->background();
      for ( QGraphicsItem * item : items ) {
        if ( item != bgitem ) {
          item->setVisible(v);
        }
      }

      update();
    }
  }
}

bool QImageSceneView::shapesVisible() const
{
  return shapesVisible_;
}

void QImageSceneView::deleteAllShapes()
{
  if ( scene_ ) {
    const QList<QGraphicsItem *> items = scene_->items();
    const QGraphicsPixmapItem * bgitem = scene_->background();
    for ( QGraphicsItem * item : items ) {
      if ( item != bgitem ) {
        delete item;
      }
    }

    update();
  }
}

void QImageSceneView::addLineShape()
{
  if ( scene_ ) {

    QPen pen(Qt::yellow);
    pen.setWidth(1);

    const QRect viewrect =
        this->rect();

    const QPointF viewCenterOnScene =
        mapToScene(viewrect.center());

    const QPoint topleft(0, 0);

    QGraphicsLineItem * item =
        scene_->addLine(0, 0,
            std::max(4, viewrect.width() / 4),
            std::max(4, viewrect.height() / 4),
            pen);

    item->setFlags(item->flags()
        | QGraphicsItem::ItemIsMovable
        /*| QGraphicsItem::ItemIsSelectable*/
        /*| QGraphicsItem::ItemIsFocusable*/);

    item->setPos(viewCenterOnScene.x() - item->line().length() / 2,
        viewCenterOnScene.y() - item->line().length() / 2);

    update();

    emit onLineShapeChanged(item);
  }

}

void QImageSceneView::addRectShape()
{
  if ( scene_ ) {

    QPen pen(Qt::yellow);
    pen.setWidth(1);

    const QRect viewrect =
        this->rect();

    const QPointF viewCenterOnScene =
        mapToScene(viewrect.center());

    const QPointF topleft(0, 0);

    const QPointF bottomright(mapToScene(QPoint(
        std::max(4, viewrect.width() / 4),
        std::max(4, viewrect.height() / 4))));

    QGraphicsRectItem * item =
        scene_->addRect(QRectF(topleft, bottomright),
            pen);

    item->setFlags(item->flags()
        | QGraphicsItem::ItemIsMovable
        /*| QGraphicsItem::ItemIsSelectable */
        /*| QGraphicsItem::ItemIsFocusable*/);

    item->setPos(viewCenterOnScene.x() - item->rect().width() / 2,
        viewCenterOnScene.y() - item->rect().height() / 2);

    update();

    emit onRectShapeChanged(item);
  }
}
