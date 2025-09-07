/*
 * QGraphicsLineShape.cc
 *
 *  Created on: Jan 16, 2023
 *      Author: amyznikov
 */

#include "QGraphicsLineShape.h"
#include "QGraphicsLineShapeSettings.h"
#include <core/debug.h>

static const QString myName = "Line";
static const QString myDescription = "Line shape";

static constexpr int hit_distance = 15;

QGraphicsLineShape::QGraphicsLineShape(QGraphicsItem *parent) :
   Base(myName, myDescription, parent)
{
}

QGraphicsLineShape::QGraphicsLineShape(const QLineF &line, QGraphicsItem *parent) :
    Base(myName, myDescription, parent),
    _line(line)
{
}

QGraphicsLineShape::QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem *parent) :
    Base(myName, myDescription, parent),
    _line(x1, y1, x2, y2)
{
}

QGraphicsLineShape::QGraphicsLineShape(const QPointF & p1, const QPointF & p2, QGraphicsItem * parent) :
    Base(myName, myDescription, parent),
    _line(p1, p2)
{
}

void QGraphicsLineShape::setLine(const QLineF &line)
{
  _line = line;
}

void QGraphicsLineShape::setLine(qreal x1, qreal y1, qreal x2, qreal y2)
{
  prepareGeometryChange();

  _line.setLine(x1, y1, x2, y2);

  updateGeometry();
  update();

  if( flags() & ItemSendsGeometryChanges ) {
    Q_EMIT itemChanged(this);
  }
}

const QLineF & QGraphicsLineShape::line() const
{
  return _line;
}

void QGraphicsLineShape::setSceneLine(const QLineF &line)
{
  prepareGeometryChange();

  _line.setPoints(mapFromScene(line.p1()),
      mapFromScene(line.p1()));

  updateGeometry();
  update();

  if( flags() & ItemSendsGeometryChanges ) {
    Q_EMIT itemChanged(this);
  }
}

void QGraphicsLineShape::setSceneLine(qreal x1, qreal y1, qreal x2, qreal y2)
{
  prepareGeometryChange();

  _line.setPoints(mapFromScene(QPointF(x1, y1)),
      mapFromScene(QPointF(x2, y2)));

  updateGeometry();
  update();

  if( flags() & ItemSendsGeometryChanges ) {
    Q_EMIT itemChanged(this);
  }
}

QLineF QGraphicsLineShape::sceneLine() const
{
  return QLineF(mapToScene(_line.p1()), mapToScene(_line.p2()));
}

void QGraphicsLineShape::setPen(const QPen & pen)
{
  if ( _pen != pen ) {
    prepareGeometryChange();
    _pen = pen;
    updateGeometry();
    update();
  }
}

void QGraphicsLineShape::setCosmeticPen(const QColor & color, int width )
{
  QPen pen(color);
  pen.setWidth(width);
  pen.setCosmetic(true);
  setPen(pen);
}

void QGraphicsLineShape::setPenWidth(int v)
{
  if ( _pen.width() != v ) {
    prepareGeometryChange();
    _pen.setWidth(v);
    updateGeometry();
    update();
  }
}

int QGraphicsLineShape::penWidth() const
{
  return _pen.width();
}

void QGraphicsLineShape::setPenColor(const QColor & color)
{
  _pen.setColor(color);
  update();
}

QColor QGraphicsLineShape::penColor() const
{
  return _pen.color();
}

const QPen & QGraphicsLineShape::pen() const
{
  return _pen;
}

void QGraphicsLineShape::setLockP1(bool v)
{
  _lockP1 = v;
}

bool QGraphicsLineShape::lockP1() const
{
  return _lockP1;
}

void QGraphicsLineShape::setLockP2(bool v)
{
  _lockP2 = v;
}

bool QGraphicsLineShape::lockP2() const
{
  return _lockP2;
}

void QGraphicsLineShape::setArrowSize(double v)
{
  _arrowSize = v;
  update();
}

double QGraphicsLineShape::arrowSize() const
{
  return _arrowSize;
}

void QGraphicsLineShape::alignVertically()
{
  prepareGeometryChange();

  const QPointF p1 =
      _line.p1();

  if( !snapToPixelGrid_ ) {
    _line.setP2(QPointF(p1.x(), _line.p2().y()));
  }
  else {
    _line.setP1(QPointF((int) p1.x(), (int) p1.y()));
    _line.setP2(QPointF((int) p1.x(), (int) _line.p2().y()));
  }

  updateGeometry();
  update();

  if( flags() & ItemSendsGeometryChanges ) {
    Q_EMIT itemChanged(this);
  }
}

void QGraphicsLineShape::alignHorizontally()
{
  prepareGeometryChange();

  const QPointF p1 =
      _line.p1();

  if( !snapToPixelGrid_ ) {
    _line.setP2(QPointF(_line.p2().x(), p1.y()));
  }
  else {
    _line.setP1(QPointF((int) p1.x(), (int) p1.y()));
    _line.setP2(QPointF((int) _line.p2().x(), (int) p1.y()));
  }

  updateGeometry();
  update();

  if( flags() & ItemSendsGeometryChanges ) {
    Q_EMIT itemChanged(this);
  }
}


QRectF QGraphicsLineShape::boundingRect() const
{
  return _boundingRect;
}

QPainterPath QGraphicsLineShape::shape() const
{
  return _shape;
}


static void compute_arrow(const QLineF & line, double arrowSize, QPointF * arrowP1, QPointF * arrowP2)
{
  const double angle = std::atan2(-line.dy(), line.dx());

  *arrowP1 = line.p2() -
      QPointF(sin(angle + M_PI / 3) * arrowSize, cos(angle + M_PI / 3) * arrowSize);

  *arrowP2 = line.p2() -
      QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize, cos(angle + M_PI - M_PI / 3) * arrowSize);
}

void QGraphicsLineShape::updateGeometry()
{
  QPainterPath path;

  path.moveTo(_line.p1());
  path.lineTo(_line.p2());

  if( _arrowSize > 1 ) {

    QPointF arrowP1, arrowP2;
    compute_arrow(_line, _arrowSize, &arrowP1, &arrowP2);

    path.moveTo(arrowP1);
    path.lineTo(_line.p2());

    path.moveTo(arrowP2);
    path.lineTo(_line.p2());
  }

  QPen pen = _pen;
  pen.setWidth(hit_distance);

  _shape =
      Base::shapeFromPath(path,
          pen);

  _boundingRect =
      _shape.boundingRect();
}

void QGraphicsLineShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  painter->setPen(_pen);
  painter->drawLine(_line);

  if( _arrowSize > 1 ) {

    QPointF arrowP1, arrowP2;

    compute_arrow(_line, _arrowSize, &arrowP1, &arrowP2);

    painter->drawLine(arrowP1, _line.p2());
    painter->drawLine(arrowP2, _line.p2());
  }
}

void QGraphicsLineShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && (flags() & ItemIsMovable) ) {

    _lastPos = e->pos();

    if( _lockP1 && _lockP2 ) {
      e->accept();
      return;
    }


    if( e->modifiers() != Qt::ControlModifier ) {
      e->accept();
      return;
    }

    QGraphicsView *view =
        getActiveView(e);

    if( view ) {

      const QPoint epos =
          view->mapFromScene(e->scenePos());

      const QPoint p1 =
          view->mapFromScene(mapToScene(_line.p1()));

      const double p1_distance =
          distance(epos, p1);

      if( p1_distance < hit_distance ) {
        currentMouseAction_ = MouseAction_MoveP1;
        e->accept();
        return;
      }

      const QPoint p2 =
          view->mapFromScene(mapToScene(_line.p2()));

      const double p2_distance =
          distance(epos, p2);

      if( p2_distance < hit_distance ) {
        currentMouseAction_ = MouseAction_MoveP2;
        e->accept();
        return;
      }

      currentMouseAction_ = MouseAction_MoveWholeLine;
      e->accept();
      return;
    }

  }

  Base::mousePressEvent(e);
}

void QGraphicsLineShape::mouseMoveEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && (flags() & ItemIsMovable) ) {

    if( _lockP1 && _lockP2 ) {
      e->accept();
      return;
    }

    switch (currentMouseAction_) {

      case MouseAction_MoveP1:
        if( !_lockP1 ) {

          prepareGeometryChange();

          if ( !snapToPixelGrid_ ) {
            _line.setP1(e->pos());
          }
          else {
            const QPointF pos = e->pos();
            _line.setP1(QPointF((int)pos.x(), (int)pos.y()));
          }

          updateGeometry();
          update();

          if( flags() & ItemSendsGeometryChanges ) {
            Q_EMIT itemChanged(this);
          }

        }
        e->ignore();
        return;

      case MouseAction_MoveP2:
        if( !_lockP2 ) {

          prepareGeometryChange();

          if ( !snapToPixelGrid_ ) {
            _line.setP2(e->pos());
          }
          else {
            const QPointF pos = e->pos();
            _line.setP2(QPointF((int)pos.x(), (int)pos.y()));
          }

          updateGeometry();
          update();

          if( flags() & ItemSendsGeometryChanges ) {
            Q_EMIT itemChanged(this);
          }
        }

        e->ignore();
        return;

      default:
        if( _lockP1 ) {

          prepareGeometryChange();

          const QPointF p1 = _line.p1();
          const QPointF epos = e->pos();

          const double angle =
              atan2(epos.y() - p1.y(), epos.x() - p1.x());

          const QPointF p2(p1.x() + cos(angle) * _line.length(),
              p1.y() + sin(angle) * _line.length());


          if ( !snapToPixelGrid_ ) {
            _line.setP2(p2);
          }
          else {
            _line.setP2(QPointF((int)p2.x(), (int)p2.y()));
          }

          updateGeometry();
          update();

          if( flags() & ItemSendsGeometryChanges ) {
            Q_EMIT itemChanged(this);
          }

        }
        else if( _lockP2 ) {

          prepareGeometryChange();

          const QPointF p2 = _line.p2();
          const QPointF epos = e->pos();

          const double angle =
              atan2(epos.y() - p2.y(), epos.x() - p2.x());

          const QPointF p1(p2.x() + cos(angle) * _line.length(),
              p2.y() + sin(angle) * _line.length());

          if ( !snapToPixelGrid_ ) {
            _line.setP1(p1);
          }
          else {
            _line.setP1(QPointF((int)p1.x(), (int)p1.y()));
          }

          updateGeometry();
          update();

          if( flags() & ItemSendsGeometryChanges ) {
            Q_EMIT itemChanged(this);
          }

        }
        else {
          currentMouseAction_ =
              MouseAction_MoveWholeLine;

          /////////////////////

          const QPointF delta =
              e->pos() - (snapToPixelGrid_ ? _lastPos : e->lastPos());

          if ( delta.x() || delta.y() ) {

            if( !snapToPixelGrid_ ) {

              prepareGeometryChange();

              _line.setP1(_line.p1() + delta);
              _line.setP2(_line.p2() + delta);

              updateGeometry();
              update();

              if( flags() & ItemSendsGeometryChanges ) {
                Q_EMIT itemChanged(this);
              }

            }
            else {

              const QPointF p1 = _line.p1();
              const QPointF p2 = _line.p2();

              const QPointF p3(qRound(p1.x() + delta.x()), qRound(p1.y() + delta.y()));
              const QPointF p4(qRound(p2.x() + delta.x()), qRound(p2.y() + delta.y()));

              if( p3.x() != p1.x() || p3.y() != p1.y() || p4.x() != p2.x() || p4.y() != p2.y() ) {

                prepareGeometryChange();

                _line.setP1(p3);
                _line.setP2(p4);
                _lastPos = e->pos();

                updateGeometry();
                update();

                if( flags() & ItemSendsGeometryChanges ) {
                  Q_EMIT itemChanged(this);
                }
              }
            }
          }
        }

        e->ignore();
        return;
    }
  }

  Base::mouseMoveEvent(e);
}

void QGraphicsLineShape::mouseReleaseEvent(QGraphicsSceneMouseEvent * e)
{
  currentMouseAction_ = MouseAction_None;
  Base::mouseReleaseEvent(e);
}

template<class Fn>
static QAction * createCheckableAction(const QString & text, bool checked, Fn && fn)
{
  QAction * action = new QAction(text);
  action->setCheckable(true);
  action->setChecked(checked);
  QObject::connect(action, &QAction::triggered,  fn);
  return action;
}

template<class Fn>
static QAction * createAction(const QString & text, Fn && fn)
{
  QAction * action = new QAction(text);
  QObject::connect(action, &QAction::triggered,  fn);
  return action;
}

void QGraphicsLineShape::popuateContextMenu(QMenu &menu, const QPoint &viewpos)
{
  menu.addAction(createCheckableAction("Options...", false,
      [this](bool checked) {
        showShapeSettings();
      }));

  menu.addSeparator();

  menu.addAction(createCheckableAction("Lock P1", _lockP1,
      [this](bool checked) {
        setLockP1(checked);
      }));

  menu.addAction(createCheckableAction("Lock P2", _lockP2,
      [this](bool checked) {
        setLockP2(checked);
      }));

  menu.addAction(createCheckableAction("Snap to pixels", snapToPixelGrid_,
      [this](bool checked) {
        setSnapToPixelGrid(checked);
      }));

  menu.addAction(createAction("Align Vertically",
      [this]() {
        alignVertically();
      }));

  menu.addAction(createAction("Align Horizontally",
      [this]() {
        alignHorizontally();
      }));

  menu.addSeparator();
  Base::popuateContextMenu(menu, viewpos);
}

void QGraphicsLineShape::showShapeSettings()
{
  QGraphicsLineShapeSettingsDialogBox dialogBox("Line Options",
      this, QApplication::activeWindow());
  dialogBox.exec();
}
