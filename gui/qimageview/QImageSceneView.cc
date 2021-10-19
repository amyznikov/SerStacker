/*
 * QImageSceneView.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageSceneView.h"
#include <core/debug.h>
//
//// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
////static double distance_from_point_to_line(const QPointF & p, const QLineF & l)
////{
////  const QPointF p1 = l.p1();
////  const QPointF p2 = l.p2();
////
////  const double x0 = p.x();
////  const double y0 = p.y();
////  const double x1 = p1.x();
////  const double y1 = p1.y();
////  const double x2 = p2.x();
////  const double y2 = p2.y();
////
////  return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
////}
//
//static double distance_from_point_to_line(const QPointF & p, const QPointF & lp1, const QPointF & lp2)
//{
//  const QPointF & p1 = lp1;
//  const QPointF p2 = lp2;
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
//
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
  const Qt::MouseButtons buttons =
      e->buttons();

  bool handled =
      false;

  if ( buttons == Qt::LeftButton && mouseScrollEnabled_ ) {

    const QPoint mpos =
        e->pos();

    QGraphicsItem * bgitem =
        scene()->background();

    const QGraphicsItem * item =
        scene()->itemAt(mapToScene(mpos),
            QTransform());

    if ( !item || item == bgitem ) {
      mouseScrollActive_ = true;
      prevMouseScrollPos_ = mpos;
      e->accept();
      handled = true;
    }
  }

//
//  if ( buttons ) {
//
//    const QImageScene * scene =
//        this->scene();
//
//    QGraphicsItem * bgitem =
//        scene->background();
//
//    const QPoint mpos =
//        e->pos();
//
//    const QPointF spos =
//        this->mapToScene(mpos);
//
//    const QList<QGraphicsItem *> items =
//        scene->items(spos);
//
//    QGraphicsItem * currentShape =
//        Q_NULLPTR;
//
//    for ( QGraphicsItem * item : items ) {
//      if ( item != bgitem ) {
//        currentShape = item;
//        break;
//      }
//    }
//
//    switch ( buttons ) {
//
//    case Qt::LeftButton : {
//      if ( currentShape ) {
//        break;
//
//        //        if ( (handled = currentShape->handleMousePressEvent(this, e)) ) {
//        //          currentShape_ = currentShape;
//        //        }
//        //        else {
//        //          currentShape_ = Q_NULLPTR;
//        //        }
//      }
//
//      if ( !handled && mouseScrollEnabled_ ) {
//
//        const QGraphicsItem * item =
//            scene->itemAt(spos,
//                QTransform());
//
//        if ( !item || item == bgitem ) {
//          mouseScrollActive_ = true;
//          prevMouseScrollPos_ = mpos;
//        }
//      }
//
//      break;
//    }
//
//    case Qt::RightButton :
//      if ( currentShape ) {
//
//        // Context menu
//        QMenu menu;
//        QAction * action;
//
//        menu.addAction(action = new QAction("Delete this shape"));
//        connect(action, &QAction::triggered,
//            [this, currentShape]() {
//              if ( currentShape == this->currentShape_ ) {
//                this->currentShape_ = Q_NULLPTR;
//              }
//
//              QGraphicsItem * item =
//                  dynamic_cast<QGraphicsItem*>(currentShape);
//
//              CF_DEBUG("item=%p", item);
//
//              if ( item ) {
//                this->scene()->removeItem(item);
//              }
//            });
//
//
//        menu.exec(mapToGlobal(mpos));
//
//      }
//      break;
//    }
//  }

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
    e->accept();
  }
//  else if ( currentShape_ ) {
//    if ( !currentShape_->handleMouseMoveEvent(this, e) ) {
//      currentShape_ =  Q_NULLPTR;
//    }
//  }
  else {
    emit onMouseMove(e);
    Base::mouseMoveEvent(e);
  }
}

void QImageSceneView::mouseReleaseEvent(QMouseEvent *e)
{
  if ( mouseScrollActive_ ) {
    mouseScrollActive_ = false;
    e->accept();
  }
//  else if ( currentShape_ ) {
//    currentShape_ =  Q_NULLPTR;
////    if ( !currentShape_->handleMouseReleaseEvent(this, e) ) {
////      currentShape_ =  Q_NULLPTR;
////    }
//  }
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

    QRectF rc(0, 0, 64, 64);

    QGraphicsLineShape * item =
        new QGraphicsLineShape(rc.x(), rc.y(),
            rc.width(), rc.height());

    item->setPen(pen);

    item->setFlags(item->flags()
        | QGraphicsItem::ItemIsMovable
        | QGraphicsItem::ItemSendsGeometryChanges
        | QGraphicsItem::ItemSendsScenePositionChanges
        /*| QGraphicsItem::ItemIsSelectable*/
        /*| QGraphicsItem::ItemIsFocusable*/);


    item->setPos(viewCenterOnScene.x() - rc.width() / 2,
        viewCenterOnScene.y() - rc.height() / 2);

    scene_->addItem(item);
    update();

    connect(item, &QGraphicsLineShape::onItemChanged,
        this, &ThisClass::onLineShapeChanged);

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

    QRectF rc(0, 0, 64, 64);

    QGraphicsRectShape * item =
        new QGraphicsRectShape(rc);

    item->setPen(pen);

    item->setFlags(item->flags()
        | QGraphicsItem::ItemIsMovable
        | QGraphicsItem::ItemSendsGeometryChanges
        | QGraphicsItem::ItemSendsScenePositionChanges
        /*| QGraphicsItem::ItemIsSelectable */
        /*| QGraphicsItem::ItemIsFocusable*/);

    item->setPos(viewCenterOnScene.x() - item->rect().width() / 2,
        viewCenterOnScene.y() - item->rect().height() / 2);

    scene_->addItem(item);
    update();

    connect(item, &QGraphicsRectShape::onItemChanged,
        this, &ThisClass::onRectShapeChanged);
    emit onRectShapeChanged(item);
  }
}
