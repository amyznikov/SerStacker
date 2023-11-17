/*
 * QGraphicsLineShape.cc
 *
 *  Created on: Jan 16, 2023
 *      Author: amyznikov
 */

#include "QGraphicsLineShape.h"
#include "QGraphicsLineShapeSettings.h"
#include <core/debug.h>

static constexpr int hit_distance = 15;

QGraphicsLineShape::QGraphicsLineShape(QGraphicsItem *parent) :
   Base(parent)
{
}

QGraphicsLineShape::QGraphicsLineShape(const QLineF &line, QGraphicsItem *parent) :
    Base(parent),
    line_(line)
{
}

QGraphicsLineShape::QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem *parent) :
    Base(parent),
    line_(x1, y1, x2, y2)
{
}

QGraphicsLineShape::QGraphicsLineShape(const QPointF & p1, const QPointF & p2, QGraphicsItem * parent) :
    Base(parent),
    line_(p1, p2)
{
}

void QGraphicsLineShape::setLine(const QLineF &line)
{
  line_ = line;
}

void QGraphicsLineShape::setLine(qreal x1, qreal y1, qreal x2, qreal y2)
{
  line_.setLine(x1, y1, x2, y2);
}

const QLineF & QGraphicsLineShape::line() const
{
  return line_;
}

void QGraphicsLineShape::setSceneLine(const QLineF &line)
{
  line_.setPoints(mapFromScene(line.p1()), mapFromScene(line.p1()));
}

void QGraphicsLineShape::setSceneLine(qreal x1, qreal y1, qreal x2, qreal y2)
{
  line_.setPoints(mapFromScene(QPointF(x1, y1)), mapFromScene(QPointF(x2, y2)));
}

QLineF QGraphicsLineShape::sceneLine() const
{
  return QLineF(mapToScene(line_.p1()), mapToScene(line_.p2()));
}

void QGraphicsLineShape::setPen(const QPen & pen)
{
  if ( pen_ != pen ) {
    prepareGeometryChange();
    pen_ = pen;
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
  if ( pen_.width() != v ) {
    prepareGeometryChange();
    pen_.setWidth(v);
    updateGeometry();
    update();
  }
}

int QGraphicsLineShape::penWidth() const
{
  return pen_.width();
}

void QGraphicsLineShape::setPenColor(const QColor & color)
{
  pen_.setColor(color);
  update();
}

QColor QGraphicsLineShape::penColor() const
{
  return pen_.color();
}

const QPen & QGraphicsLineShape::pen() const
{
  return pen_;
}

void QGraphicsLineShape::setLockP1(bool v)
{
  lockP1_ = v;
}

bool QGraphicsLineShape::lockP1() const
{
  return lockP1_;
}

void QGraphicsLineShape::setLockP2(bool v)
{
  lockP2_ = v;
}

bool QGraphicsLineShape::lockP2() const
{
  return lockP2_;
}

void QGraphicsLineShape::setArrowSize(double v)
{
  arrowSize_ = v;
  update();
}

double QGraphicsLineShape::arrowSize() const
{
  return arrowSize_;
}

void QGraphicsLineShape::alignVertically()
{
  prepareGeometryChange();

  const QPointF p1 =
      line_.p1();

  if( !snapToPixelGrid_ ) {
    line_.setP2(QPointF(p1.x(), line_.p2().y()));
  }
  else {
    line_.setP1(QPointF((int) p1.x(), (int) p1.y()));
    line_.setP2(QPointF((int) p1.x(), (int) line_.p2().y()));
  }

  updateGeometry();
  update();
}

void QGraphicsLineShape::alignHorizontally()
{
  prepareGeometryChange();

  const QPointF p1 =
      line_.p1();

  if( !snapToPixelGrid_ ) {
    line_.setP2(QPointF(line_.p2().x(), p1.y()));
  }
  else {
    line_.setP1(QPointF((int) p1.x(), (int) p1.y()));
    line_.setP2(QPointF((int) line_.p2().x(), (int) p1.y()));
  }

  updateGeometry();
  update();
}


QRectF QGraphicsLineShape::boundingRect() const
{
  return boundingRect_;
}

QPainterPath QGraphicsLineShape::shape() const
{
  return shape_;
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

  path.moveTo(line_.p1());
  path.lineTo(line_.p2());

  if( arrowSize_ > 1 ) {

    QPointF arrowP1, arrowP2;
    compute_arrow(line_, arrowSize_, &arrowP1, &arrowP2);

    path.moveTo(arrowP1);
    path.lineTo(line_.p2());

    path.moveTo(arrowP2);
    path.lineTo(line_.p2());
  }

  QPen pen = pen_;
  pen.setWidth(hit_distance);

  shape_ =
      Base::shapeFromPath(path,
          pen);

  boundingRect_ =
      shape_.boundingRect();
}

void QGraphicsLineShape::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
  Base::paint(painter, option, widget);

  painter->setPen(pen_);
  painter->drawLine(line_);

  if( arrowSize_ > 1 ) {

    QPointF arrowP1, arrowP2;

    compute_arrow(line_, arrowSize_, &arrowP1, &arrowP2);

    painter->drawLine(arrowP1, line_.p2());
    painter->drawLine(arrowP2, line_.p2());
  }
}

void QGraphicsLineShape::mousePressEvent(QGraphicsSceneMouseEvent * e)
{
  if( e->buttons() == Qt::LeftButton && (flags() & ItemIsMovable) ) {

    if( lockP1_ && lockP2_ ) {
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
          view->mapFromScene(mapToScene(line_.p1()));

      const double p1_distance =
          distance(epos, p1);

      if( p1_distance < hit_distance ) {
        currentMouseAction_ = MouseAction_MoveP1;
        e->accept();
        return;
      }

      const QPoint p2 =
          view->mapFromScene(mapToScene(line_.p2()));

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

    if( lockP1_ && lockP2_ ) {
      e->accept();
      return;
    }

    switch (currentMouseAction_) {

      case MouseAction_MoveP1:
        if( !lockP1_ ) {

          prepareGeometryChange();

          if ( !snapToPixelGrid_ ) {
            line_.setP1(e->pos());
          }
          else {
            const QPointF pos = e->pos();
            line_.setP1(QPointF((int)pos.x(), (int)pos.y()));
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
        if( !lockP2_ ) {

          prepareGeometryChange();

          if ( !snapToPixelGrid_ ) {
            line_.setP2(e->pos());
          }
          else {
            const QPointF pos = e->pos();
            line_.setP2(QPointF((int)pos.x(), (int)pos.y()));
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
        if( lockP1_ ) {

          prepareGeometryChange();

          const QPointF p1 = line_.p1();
          const QPointF epos = e->pos();

          const double angle =
              atan2(epos.y() - p1.y(), epos.x() - p1.x());

          const QPointF p2(p1.x() + cos(angle) * line_.length(),
              p1.y() + sin(angle) * line_.length());


          if ( !snapToPixelGrid_ ) {
            line_.setP2(p2);
          }
          else {
            line_.setP2(QPointF((int)p2.x(), (int)p2.y()));
          }

          updateGeometry();
          update();

          if( flags() & ItemSendsGeometryChanges ) {
            Q_EMIT itemChanged(this);
          }

          e->ignore();
          return;
        }

        if( lockP2_ ) {

          prepareGeometryChange();

          const QPointF p2 = line_.p2();
          const QPointF epos = e->pos();

          const double angle =
              atan2(epos.y() - p2.y(), epos.x() - p2.x());

          const QPointF p1(p2.x() + cos(angle) * line_.length(),
              p2.y() + sin(angle) * line_.length());

          if ( !snapToPixelGrid_ ) {
            line_.setP1(p1);
          }
          else {
            line_.setP1(QPointF((int)p1.x(), (int)p1.y()));
          }

          updateGeometry();
          update();

          if( flags() & ItemSendsGeometryChanges ) {
            Q_EMIT itemChanged(this);
          }

          e->ignore();
          return;
        }

        currentMouseAction_ =
            MouseAction_MoveWholeLine;

        Base::mouseMoveEvent(e);
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

bool QGraphicsLineShape::popuateContextMenu(const QGraphicsSceneContextMenuEvent * e, QMenu & menu)
{
  menu.addAction(createCheckableAction("Lock P1", lockP1_,
      [this](bool checked) {
        setLockP1(checked);
      }));

  menu.addAction(createCheckableAction("Lock P2", lockP2_,
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

  menu.addAction(createCheckableAction("Options...", false,
      [this](bool checked) {
        showShapeSettings();
      }));

  menu.addSeparator();
  Base::popuateContextMenu(e, menu);

  return true;
}

void QGraphicsLineShape::showShapeSettings()
{
  QGraphicsLineShapeSettingsDialogBox dialogBox("Line Options",
      this, QApplication::activeWindow());
  dialogBox.exec();
}


