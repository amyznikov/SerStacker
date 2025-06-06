/*
 * QGraphicsRectShape.cc
 *
 *  Created on: Jan 4, 2023
 *      Author: amyznikov
 */

#include "QGraphicsRectShape.h"
#include "QGraphicsRectShapeSettings.h"
#include <float.h>
#include <limits.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<QGraphicsRectShape::MouseAction>()
{
  static const c_enum_member members[] = {
      { QGraphicsRectShape::MouseAction_None, "None", "" },
      { QGraphicsRectShape::MouseAction_MoveRect, "MoveRect", "" },
      { QGraphicsRectShape::MouseAction_MoveTop, "MoveTop", "" },
      { QGraphicsRectShape::MouseAction_MoveRight, "MoveRight", "" },
      { QGraphicsRectShape::MouseAction_MoveBottom, "MoveBottom", "" },
      { QGraphicsRectShape::MouseAction_MoveLeft, "MoveLeft", "" },
      { QGraphicsRectShape::MouseAction_MoveTopLeft, "MoveTopLeft", "" },
      { QGraphicsRectShape::MouseAction_MoveTopRight, "MoveTopRight", "" },
      { QGraphicsRectShape::MouseAction_MoveBottomRight, "MoveBottomRight", "" },
      { QGraphicsRectShape::MouseAction_MoveBottomLeft, "MoveBottomLeft", "" },
      { QGraphicsRectShape::MouseAction_None}
  };

  return members;
}


QGraphicsRectShape::QGraphicsRectShape(QGraphicsItem * parent) :
    Base(parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QRectF & rect, QGraphicsItem * parent) :
    Base(parent),
    rect_(rect)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QString & name, const QString & description, QGraphicsItem * parent) :
    Base(name, description, parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QString & name, const QString & description, const QRectF & rect,
    QGraphicsItem * parent) :
    Base(name, description, parent),
    rect_(rect)
{
}

void QGraphicsRectShape::setRect(const QRectF & rc)
{
  if (rect_ != rc ) {

    prepareGeometryChange();
    rect_ = rc;
    updateGeometry();
    update();

    if( flags() & ItemSendsGeometryChanges ) {
      Q_EMIT itemChanged(this);
    }
  }
}

const QRectF & QGraphicsRectShape::rect() const
{
  return rect_;
}

QRectF QGraphicsRectShape::sceneRect() const
{
  return mapToScene(rect_).boundingRect();
}

QRect QGraphicsRectShape::iSceneRect() const
{
  const QRectF sceneRect = this->sceneRect();
  return QRect((int) (sceneRect.x()), (int) (sceneRect.y()), (int) (sceneRect.width()), (int) (sceneRect.height()));
}

void QGraphicsRectShape::setCenter(const QPointF & p)
{
  if (rect_.center() != p ) {

    prepareGeometryChange();
    rect_.moveCenter(p);
    updateGeometry();
    update();

    if( flags() & ItemSendsGeometryChanges ) {
      Q_EMIT itemChanged(this);
    }
  }
}

QPointF QGraphicsRectShape::center() const
{
  return rect_.center();
}

void QGraphicsRectShape::setPen(const QPen & pen)
{
  if ( pen_ != pen ) {
    prepareGeometryChange();
    pen_ = pen;
    updateGeometry();
    update();
  }
}

void QGraphicsRectShape::setCosmeticPen(const QColor & color, int width )
{
  QPen pen(color);
  pen.setWidth(width);
  pen.setCosmetic(true);
  setPen(pen);
}

void QGraphicsRectShape::setPenWidth(int v)
{
  if ( pen_.width() != v ) {
    prepareGeometryChange();
    pen_.setWidth(v);
    updateGeometry();
    update();
  }
}

int QGraphicsRectShape::penWidth() const
{
  return pen_.width();
}

void QGraphicsRectShape::setPenColor(const QColor & color)
{
  pen_.setColor(color);
  update();
}

QColor QGraphicsRectShape::penColor() const
{
  return pen_.color();
}

const QPen & QGraphicsRectShape::pen() const
{
  return pen_;
}

void QGraphicsRectShape::setBrush(const QBrush & brush)
{
  if ( brush_ != brush ) {
    prepareGeometryChange();
    brush_ = brush;
    // updateGeometry();
    update();
  }
}

const QBrush & QGraphicsRectShape::brush() const
{
  return brush_;
}

void QGraphicsRectShape::setResizable(bool v)
{
  itemIsResizable_ = v;
}

bool QGraphicsRectShape::resizable() const
{
  return itemIsResizable_;
}

void QGraphicsRectShape::updateGeometry()
{
  QPainterPath path;

  boundingRect_ =
      rect_.adjusted(-hitDstance_, -hitDstance_,
          +hitDstance_, +hitDstance_);


  path.addRect(boundingRect_);

  shape_ =
      Base::shapeFromPath(path, pen());
}

QRectF QGraphicsRectShape::boundingRect() const
{
  return boundingRect_;
}

QPainterPath QGraphicsRectShape::shape() const
{
  return shape_;
}

void QGraphicsRectShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  if( !rect_.isEmpty() ) {

    painter->setPen(pen_);
    painter->setBrush(brush_);

    painter->drawRect(rect_);
  }
}

void QGraphicsRectShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {

    currentMouseAction_  = MouseAction_None;
    mdelta_ = e->pos() - rect_.topLeft();

    if( itemIsResizable_ && e->modifiers() == Qt::ControlModifier) {

      QGraphicsView *view =
          getActiveView(e);

      if( view ) {

        const QPoint viewpos =
            view->mapFromScene(e->scenePos());

        const QPoint corners[4] = {
            view->mapFromScene(mapToScene(rect_.topLeft())),
            view->mapFromScene(mapToScene(rect_.topRight())),
            view->mapFromScene(mapToScene(rect_.bottomRight())),
            view->mapFromScene(mapToScene(rect_.bottomLeft())),
        };

        // Try corners first

        double best_corner_distance = DBL_MAX;
        int best_corner = -1;

        for( int i = 0; i < 4; ++i ) {

          const double d =
              distance(viewpos,
                  corners[i]);

          if( d < hitDstance_ && d < best_corner_distance ) {
            best_corner_distance = d;
            best_corner = i;
          }
        }

        if( best_corner != -1 ) {

          switch (best_corner) {
            case 0:
              currentMouseAction_ = MouseAction_MoveTopLeft;
              break;
            case 1:
              currentMouseAction_ = MouseAction_MoveTopRight;
              break;
            case 2:
              currentMouseAction_ = MouseAction_MoveBottomRight;
              break;
            case 3:
              currentMouseAction_ = MouseAction_MoveBottomLeft;
              break;
          }

          e->accept();
          return;
        }

        // Try side borders now

        double best_side_distance = DBL_MAX;
        int best_side = -1;

        for( int i1 = 0; i1 < 4; ++i1 ) {

          const int i2 =
              i1 < 3 ? i1 + 1 :
                  0;

          const double d =
              distance_from_point_to_line(viewpos,
                  corners[i1], corners[i2]);

          if( d < hitDstance_ && d < best_side_distance ) {
            best_side_distance = d;
            best_side = i1;
          }
        }

        if( best_side != -1 ) {

          switch (best_side) {
            case 0:
              currentMouseAction_ = MouseAction_MoveTop;
              break;
            case 1:
              currentMouseAction_ = MouseAction_MoveRight;
              break;
            case 2:
              currentMouseAction_ = MouseAction_MoveBottom;
              break;
            case 3:
              currentMouseAction_ = MouseAction_MoveLeft;
              break;
          }

          e->accept();
          return;
        }
      }

      if( !fixOnSceneCenter_ && (flags() & ItemIsMovable) ) {
        currentMouseAction_ = MouseAction_MoveRect;
      }

      e->accept();
      return;
    }
  }

  // Try move
  if( !fixOnSceneCenter_ ) {
    if( (flags() & ItemIsMovable) && (e->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier)) ) {
      Base::mousePressEvent(e);
      e->accept();
      return;
    }
  }

  e->ignore();
}

void QGraphicsRectShape::mouseMoveEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {

    if( (flags() & ItemIsMovable) && e->modifiers() == Qt::ShiftModifier ) {
      currentMouseAction_ = MouseAction_MoveRect;
    }

    if ( currentMouseAction_ != MouseAction_None ) {

      const QRectF oldrect =
          this->rect_;

      if( currentMouseAction_ == MouseAction_MoveRect ) {
        if( !snapToPixelGrid_ ) {
          rect_.moveTo(e->pos() - mdelta_);
        }
        else {
          rect_.moveTo(round(e->pos().x() - mdelta_.x()),
              round(e->pos().y() - mdelta_.y()));
        }
      }
      else {

        const QPointF pos = snapToPixelGrid_ ?
            QPointF(round(e->pos().x()), round(e->pos().y())) :
            e->pos();

        switch (currentMouseAction_) {
          case MouseAction_MoveTop:
            if( pos.y() < rect_.bottom() - 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setTop(pos.y());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setTop(pos.y());
                rect_.setBottom(2 * center.y() - pos.y());
              }
            }
            break;
          case MouseAction_MoveRight:
            if( pos.x() > rect_.left() + 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setRight(pos.x());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setRight(pos.x());
                rect_.setLeft(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveBottom:
            if( pos.y() > rect_.top() + 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setBottom(pos.y());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setBottom(pos.y());
                rect_.setTop(2 * center.y() - pos.y());
              }
            }
            break;
          case MouseAction_MoveLeft:
            if( pos.x() <= rect_.right() - 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setLeft(pos.x());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setLeft(pos.x());
                rect_.setRight(2 * center.x() - pos.x());
              }
            }
            break;

          case MouseAction_MoveTopLeft:
            if( pos.y() < rect_.bottom() - 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setTop(pos.y());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setTop(pos.y());
                rect_.setBottom(2 * center.y() - pos.y());
              }
            }
            if( pos.x() < rect_.right() - 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setLeft(pos.x());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setLeft(pos.x());
                rect_.setRight(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveTopRight:
            if( pos.y() < rect_.bottom() - 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setTop(pos.y());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setTop(pos.y());
                rect_.setBottom(2 * center.y() - pos.y());
              }
            }
            if( pos.x() >= rect_.left() + 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setRight(pos.x());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setRight(pos.x());
                rect_.setLeft(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveBottomRight:
            if( pos.y() > rect_.top() + 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setBottom(pos.y());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setBottom(pos.y());
                rect_.setTop(2 * center.y() - pos.y());
              }
            }
            if( pos.x() >= rect_.left() + 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setRight(pos.x());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setRight(pos.x());
                rect_.setLeft(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveBottomLeft:
            if( pos.y() >= rect_.top() + 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setBottom(pos.y());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setBottom(pos.y());
                rect_.setTop(2 * center.y() - pos.y());
              }
            }
            if( pos.x() <= rect_.right() - 1 ) {
              if( !fixOnSceneCenter_ ) {
                rect_.setLeft(pos.x());
              }
              else {
                const QPointF center = rect_.center();
                rect_.setLeft(pos.x());
                rect_.setRight(2 * center.x() - pos.x());
              }
            }
            break;
        }
      }

      if( rect_ != oldrect ) {

        prepareGeometryChange();

        if ( rect_.width() < 1 ) {
          const QPointF center = rect_.center();
          rect_.setWidth(1);
          rect_.moveCenter(center);
        }
        if ( rect_.height() < 1 ) {
          const QPointF center = rect_.center();
          rect_.setHeight(1);
          rect_.moveCenter(center);
        }

        updateGeometry();
        update();

        if( (flags() & (ItemSendsGeometryChanges | ItemSendsScenePositionChanges)) ) {
          Q_EMIT itemChanged(this);
        }
      }
    }

    e->ignore();
    return;
  }

  Base::mouseMoveEvent(e);
}

void QGraphicsRectShape::mouseReleaseEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {
    currentMouseAction_ = MouseAction_None;
  }
  Base::mouseReleaseEvent(e);
}


bool QGraphicsRectShape::fixOnSceneCenter() const
{
  return fixOnSceneCenter_;
}

void QGraphicsRectShape::setFixOnSceneCenter(bool v)
{
  if( fixOnSceneCenter_ != v ) {

    fixOnSceneCenter_ = v;

    QGraphicsScene *scene =
        this->scene();

    if( scene ) {

      if( !fixOnSceneCenter_ ) {
        scene->disconnect(this);
      }
      else {
        connect(scene, &QGraphicsScene::sceneRectChanged,
            this, &ThisClass::onSceneRectChanged);
      }

      if( fixOnSceneCenter_ ) {
        setCenter(mapFromScene(scene->sceneRect().center()));
      }
    }
  }
}

void QGraphicsRectShape::onSceneChange()
{
  QGraphicsScene * scene =
      this->scene();

  if ( scene ) {
    scene->disconnect(this);
  }
}

void QGraphicsRectShape::onSceneHasChanged()
{
  if( fixOnSceneCenter_ ) {

    QGraphicsScene *scene =
        this->scene();

    if( scene ) {
      connect(scene, &QGraphicsScene::sceneRectChanged,
          this, &ThisClass::onSceneRectChanged);
    }
  }
}

void QGraphicsRectShape::onSceneRectChanged(const QRectF &rect)
{
  if ( fixOnSceneCenter_ ) {
    setCenter(mapFromScene(rect.center()));
  }
}

bool QGraphicsRectShape::popuateContextMenu(const QGraphicsSceneContextMenuEvent * e, QMenu & menu)
{
  QAction * action;
  QMenu * subMenu;
  QString copyText;


//  if ( !showSettingsAction_ ) {
//
//    showSettingsAction_ = new QAction("Options...", this);
//
//    connect(showSettingsAction_, &QAction::triggered,
//        this, &ThisClass::showShapeSettings);
//  }


  menu.addSeparator();

  menu.addAction(action = new QAction("Options..."));
  connect(action, &QAction::triggered,
      this, &ThisClass::showShapeSettings);

  menu.addSeparator();

  menu.addAction(action = new QAction("Center on scene"));
  action->setCheckable(true);
  action->setChecked(fixOnSceneCenter_);
  connect(action, &QAction::triggered,
      [this](bool checked) {
        setFixOnSceneCenter(checked);
      });

  menu.addAction(action = new QAction("Fix Size"));
  action->setCheckable(true);
  action->setChecked(!resizable());
  connect(action, &QAction::triggered,
      [this](bool checked) {
        setResizable(!checked);
      });

  menu.addSeparator();

  subMenu =
      menu.addMenu("Copy");

  copyText = qsprintf("%gx%g", rect_.width(), rect_.height());
  subMenu->addAction(copyText,
      [copyText]() {
        QApplication::clipboard()->setText(copyText);
      });

  copyText = qsprintf("%d;%d;%dx%d", (int) rect_.x(), (int) rect_.y(), (int) rect_.width(), (int) rect_.height());
  subMenu->addAction(copyText,
      [copyText]() {
        QApplication::clipboard()->setText(copyText);
      });

  if( rect_.width() != (int) rect_.width() || rect_.height() != (int) rect_.height() ||
      rect_.x() != (int) rect_.x() || rect_.y() != (int) rect_.y() ) {

    copyText = qsprintf("%g;%g;%gx%g", rect_.x(), rect_.y(), rect_.width(), rect_.height());
    subMenu->addAction(copyText,
        [copyText]() {
          QApplication::clipboard()->setText(copyText);
        });
  }

  copyText = qsprintf("%g;%g;%g;%g", rect_.left(), rect_.top(), rect_.right(), rect_.bottom());
  subMenu->addAction(copyText,
      [copyText]() {
        QApplication::clipboard()->setText(copyText);
      });

  menu.addSeparator();
  Base::popuateContextMenu(e, menu);

  return true;

}

void QGraphicsRectShape::showShapeSettings()
{
  QGraphicsRectShapeSettingsDialogBox dialogBox("Rectangle Options",
      this, QApplication::activeWindow());

  dialogBox.exec();
}

