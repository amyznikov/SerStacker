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


static const QString myName = "Rect";
static const QString myDescription = "Rect shape";

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
    Base(myName, myDescription, parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QRectF & rect, QGraphicsItem * parent) :
    Base(myName, myDescription, parent),
    _rect(rect)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QString & name, const QString & description, QGraphicsItem * parent) :
    Base(name, description, parent)
{
}

QGraphicsRectShape::QGraphicsRectShape(const QString & name, const QString & description, const QRectF & rect,
    QGraphicsItem * parent) :
    Base(name, description, parent),
    _rect(rect)
{
}

void QGraphicsRectShape::setRect(const QRectF & rc)
{
  if (_rect != rc ) {
    prepareGeometryChange();
    _rect = rc;
    updateGeometry();
    update();
  }
}

void QGraphicsRectShape::setSceneRect(const QPointF & topLeft, const QPointF & bottomRight)
{
  setRect(QRectF(mapFromScene(topLeft), mapFromScene(bottomRight)));
}

const QRectF & QGraphicsRectShape::rect() const
{
  return _rect;
}

QRectF QGraphicsRectShape::sceneRect() const
{
  return mapToScene(_rect).boundingRect();
}

QRect QGraphicsRectShape::iSceneRect() const
{
  const QRectF sceneRect = this->sceneRect();
  return QRect((int) (sceneRect.x()), (int) (sceneRect.y()), (int) (sceneRect.width()), (int) (sceneRect.height()));
}

void QGraphicsRectShape::setCenter(const QPointF & p)
{
  if (_rect.center() != p ) {
    prepareGeometryChange();
    _rect.moveCenter(p);
    updateGeometry();
    update();
  }
}

QPointF QGraphicsRectShape::center() const
{
  return _rect.center();
}

void QGraphicsRectShape::setPen(const QPen & pen)
{
  if ( _pen != pen ) {
    prepareGeometryChange();
    _pen = pen;
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
  if ( _pen.width() != v ) {
    prepareGeometryChange();
    _pen.setWidth(v);
    updateGeometry();
    update();
  }
}

int QGraphicsRectShape::penWidth() const
{
  return _pen.width();
}

void QGraphicsRectShape::setPenColor(const QColor & color)
{
  _pen.setColor(color);
  update();
}

QColor QGraphicsRectShape::penColor() const
{
  return _pen.color();
}

const QPen & QGraphicsRectShape::pen() const
{
  return _pen;
}

void QGraphicsRectShape::setBrush(const QBrush & brush)
{
  if ( _brush != brush ) {
    prepareGeometryChange();
    _brush = brush;
    // updateGeometry();
    update();
  }
}

const QBrush & QGraphicsRectShape::brush() const
{
  return _brush;
}

void QGraphicsRectShape::setResizable(bool v)
{
  _itemIsResizable = v;
}

bool QGraphicsRectShape::resizable() const
{
  return _itemIsResizable;
}

void QGraphicsRectShape::updateGeometry()
{
  QPainterPath path;

  _boundingRect =
      _rect.adjusted(-_hitDstance, -_hitDstance,
          +_hitDstance, +_hitDstance);


  path.addRect(_boundingRect);

  _shape =
      Base::shapeFromPath(path, pen());
}

QRectF QGraphicsRectShape::boundingRect() const
{
  return _boundingRect;
}

QPainterPath QGraphicsRectShape::shape() const
{
  return _shape;
}

void QGraphicsRectShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  if( !_rect.isEmpty() ) {

    painter->setPen(_pen);
    painter->setBrush(_brush);

    painter->drawRect(_rect);
  }
}

void QGraphicsRectShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton ) {

    _currentMouseAction  = MouseAction_None;
    _mdelta = e->pos() - _rect.topLeft();

    if( _itemIsResizable && e->modifiers() == Qt::ControlModifier) {

      QGraphicsView *view =
          getActiveView(e);

      if( view ) {

        const QPoint viewpos =
            view->mapFromScene(e->scenePos());

        const QPoint corners[4] = {
            view->mapFromScene(mapToScene(_rect.topLeft())),
            view->mapFromScene(mapToScene(_rect.topRight())),
            view->mapFromScene(mapToScene(_rect.bottomRight())),
            view->mapFromScene(mapToScene(_rect.bottomLeft())),
        };

        // Try corners first

        double best_corner_distance = DBL_MAX;
        int best_corner = -1;

        for( int i = 0; i < 4; ++i ) {

          const double d =
              distance(viewpos,
                  corners[i]);

          if( d < _hitDstance && d < best_corner_distance ) {
            best_corner_distance = d;
            best_corner = i;
          }
        }

        if( best_corner != -1 ) {

          switch (best_corner) {
            case 0:
              _currentMouseAction = MouseAction_MoveTopLeft;
              break;
            case 1:
              _currentMouseAction = MouseAction_MoveTopRight;
              break;
            case 2:
              _currentMouseAction = MouseAction_MoveBottomRight;
              break;
            case 3:
              _currentMouseAction = MouseAction_MoveBottomLeft;
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

          if( d < _hitDstance && d < best_side_distance ) {
            best_side_distance = d;
            best_side = i1;
          }
        }

        if( best_side != -1 ) {

          switch (best_side) {
            case 0:
              _currentMouseAction = MouseAction_MoveTop;
              break;
            case 1:
              _currentMouseAction = MouseAction_MoveRight;
              break;
            case 2:
              _currentMouseAction = MouseAction_MoveBottom;
              break;
            case 3:
              _currentMouseAction = MouseAction_MoveLeft;
              break;
          }

          e->accept();
          return;
        }
      }

      if( !_fixOnSceneCenter && (flags() & ItemIsMovable) ) {
        _currentMouseAction = MouseAction_MoveRect;
      }

      e->accept();
      return;
    }
  }

  // Try move
  if( !_fixOnSceneCenter ) {
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
      _currentMouseAction = MouseAction_MoveRect;
    }

    if ( _currentMouseAction != MouseAction_None ) {

      const QRectF oldrect =
          this->_rect;

      if( _currentMouseAction == MouseAction_MoveRect ) {
        if( !snapToPixelGrid_ ) {
          _rect.moveTo(e->pos() - _mdelta);
        }
        else {
          _rect.moveTo(round(e->pos().x() - _mdelta.x()),
              round(e->pos().y() - _mdelta.y()));
        }
      }
      else {

        const QPointF pos = snapToPixelGrid_ ?
            QPointF(round(e->pos().x()), round(e->pos().y())) :
            e->pos();

        switch (_currentMouseAction) {
          case MouseAction_MoveTop:
            if( pos.y() < _rect.bottom() - 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setTop(pos.y());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setTop(pos.y());
                _rect.setBottom(2 * center.y() - pos.y());
              }
            }
            break;
          case MouseAction_MoveRight:
            if( pos.x() > _rect.left() + 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setRight(pos.x());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setRight(pos.x());
                _rect.setLeft(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveBottom:
            if( pos.y() > _rect.top() + 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setBottom(pos.y());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setBottom(pos.y());
                _rect.setTop(2 * center.y() - pos.y());
              }
            }
            break;
          case MouseAction_MoveLeft:
            if( pos.x() <= _rect.right() - 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setLeft(pos.x());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setLeft(pos.x());
                _rect.setRight(2 * center.x() - pos.x());
              }
            }
            break;

          case MouseAction_MoveTopLeft:
            if( pos.y() < _rect.bottom() - 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setTop(pos.y());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setTop(pos.y());
                _rect.setBottom(2 * center.y() - pos.y());
              }
            }
            if( pos.x() < _rect.right() - 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setLeft(pos.x());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setLeft(pos.x());
                _rect.setRight(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveTopRight:
            if( pos.y() < _rect.bottom() - 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setTop(pos.y());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setTop(pos.y());
                _rect.setBottom(2 * center.y() - pos.y());
              }
            }
            if( pos.x() >= _rect.left() + 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setRight(pos.x());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setRight(pos.x());
                _rect.setLeft(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveBottomRight:
            if( pos.y() > _rect.top() + 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setBottom(pos.y());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setBottom(pos.y());
                _rect.setTop(2 * center.y() - pos.y());
              }
            }
            if( pos.x() >= _rect.left() + 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setRight(pos.x());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setRight(pos.x());
                _rect.setLeft(2 * center.x() - pos.x());
              }
            }
            break;
          case MouseAction_MoveBottomLeft:
            if( pos.y() >= _rect.top() + 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setBottom(pos.y());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setBottom(pos.y());
                _rect.setTop(2 * center.y() - pos.y());
              }
            }
            if( pos.x() <= _rect.right() - 1 ) {
              if( !_fixOnSceneCenter ) {
                _rect.setLeft(pos.x());
              }
              else {
                const QPointF center = _rect.center();
                _rect.setLeft(pos.x());
                _rect.setRight(2 * center.x() - pos.x());
              }
            }
            break;
        }
      }

      if( _rect != oldrect ) {

        prepareGeometryChange();

        if ( _rect.width() < 1 ) {
          const QPointF center = _rect.center();
          _rect.setWidth(1);
          _rect.moveCenter(center);
        }
        if ( _rect.height() < 1 ) {
          const QPointF center = _rect.center();
          _rect.setHeight(1);
          _rect.moveCenter(center);
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
    _currentMouseAction = MouseAction_None;
  }
  Base::mouseReleaseEvent(e);
}


bool QGraphicsRectShape::fixOnSceneCenter() const
{
  return _fixOnSceneCenter;
}

void QGraphicsRectShape::setFixOnSceneCenter(bool v)
{
  if( _fixOnSceneCenter != v ) {

    _fixOnSceneCenter = v;

    QGraphicsScene *scene =
        this->scene();

    if( scene ) {

      if( !_fixOnSceneCenter ) {
        scene->disconnect(this);
      }
      else {
        connect(scene, &QGraphicsScene::sceneRectChanged,
            this, &ThisClass::onSceneRectChanged);
      }

      if( _fixOnSceneCenter ) {
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
  if( _fixOnSceneCenter ) {

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
  if ( _fixOnSceneCenter ) {
    setCenter(mapFromScene(rect.center()));
  }
}

//bool QGraphicsRectShape::popuateContextMenu(const QGraphicsSceneContextMenuEvent * e, QMenu & menu)
//{
//  QAction * action;
//  QMenu * subMenu;
//  QString copyText;
//
//
////  if ( !showSettingsAction_ ) {
////
////    showSettingsAction_ = new QAction("Options...", this);
////
////    connect(showSettingsAction_, &QAction::triggered,
////        this, &ThisClass::showShapeSettings);
////  }
//
//
//  menu.addSeparator();
//
//  menu.addAction(action = new QAction("Options..."));
//  connect(action, &QAction::triggered,
//      this, &ThisClass::showShapeSettings);
//
//  menu.addSeparator();
//
//  menu.addAction(action = new QAction("Center on scene"));
//  action->setCheckable(true);
//  action->setChecked(fixOnSceneCenter_);
//  connect(action, &QAction::triggered,
//      [this](bool checked) {
//        setFixOnSceneCenter(checked);
//      });
//
//  menu.addAction(action = new QAction("Fix Size"));
//  action->setCheckable(true);
//  action->setChecked(!resizable());
//  connect(action, &QAction::triggered,
//      [this](bool checked) {
//        setResizable(!checked);
//      });
//
//  menu.addSeparator();
//
//  subMenu =
//      menu.addMenu("Copy");
//
//  copyText = qsprintf("%gx%g", rect_.width(), rect_.height());
//  subMenu->addAction(copyText,
//      [copyText]() {
//        QApplication::clipboard()->setText(copyText);
//      });
//
//  copyText = qsprintf("%d;%d;%dx%d", (int) rect_.x(), (int) rect_.y(), (int) rect_.width(), (int) rect_.height());
//  subMenu->addAction(copyText,
//      [copyText]() {
//        QApplication::clipboard()->setText(copyText);
//      });
//
//  if( rect_.width() != (int) rect_.width() || rect_.height() != (int) rect_.height() ||
//      rect_.x() != (int) rect_.x() || rect_.y() != (int) rect_.y() ) {
//
//    copyText = qsprintf("%g;%g;%gx%g", rect_.x(), rect_.y(), rect_.width(), rect_.height());
//    subMenu->addAction(copyText,
//        [copyText]() {
//          QApplication::clipboard()->setText(copyText);
//        });
//  }
//
//  copyText = qsprintf("%g;%g;%g;%g", rect_.left(), rect_.top(), rect_.right(), rect_.bottom());
//  subMenu->addAction(copyText,
//      [copyText]() {
//        QApplication::clipboard()->setText(copyText);
//      });
//
//  menu.addSeparator();
//  Base::popuateContextMenu(e, menu);
//
//  return true;
//
//}

void QGraphicsRectShape::showShapeSettings()
{
  QGraphicsRectShapeSettingsDialogBox dialogBox("Rectangle Options",
      this, QApplication::activeWindow());

  dialogBox.exec();
}

void QGraphicsRectShape::popuateContextMenu(QMenu & menu, const QPoint & viewpos)
{
  QAction * action;
  QMenu * subMenu;
  QString copyText;

  menu.addAction(action = new QAction("Options..."));
  connect(action, &QAction::triggered,
      this, &ThisClass::showShapeSettings);

  menu.addSeparator();

  menu.addAction(action = new QAction("Center on scene"));
  action->setCheckable(true);
  action->setChecked(_fixOnSceneCenter);
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

  subMenu = menu.addMenu("Copy");

  copyText = qsprintf("%gx%g", _rect.width(), _rect.height());
  subMenu->addAction(copyText,
      [copyText]() {
        QApplication::clipboard()->setText(copyText);
      });

  copyText = qsprintf("%d;%d;%dx%d", (int) _rect.x(), (int) _rect.y(), (int) _rect.width(), (int) _rect.height());
  subMenu->addAction(copyText,
      [copyText]() {
        QApplication::clipboard()->setText(copyText);
      });

  if( std::abs(_rect.width()-(int) _rect.width()) || std::abs(_rect.height() -(int) _rect.height()) ||
      std::abs(_rect.x() -(int) _rect.x()) || std::abs(_rect.y() -(int) _rect.y()) ) {

    copyText = qsprintf("%g;%g;%gx%g", _rect.x(), _rect.y(), _rect.width(), _rect.height());
    subMenu->addAction(copyText,
        [copyText]() {
          QApplication::clipboard()->setText(copyText);
        });
  }

  copyText = qsprintf("%g;%g;%g;%g", _rect.left(), _rect.top(), _rect.right(), _rect.bottom());
  subMenu->addAction(copyText,
      [copyText]() {
        QApplication::clipboard()->setText(copyText);
      });

  menu.addSeparator();
  Base::popuateContextMenu(menu, viewpos);
}
