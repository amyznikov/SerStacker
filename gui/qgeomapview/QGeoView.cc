/*
 * QGeoView.cc
 *
 *  Created on: Nov 25, 2022
 *      Author: amyznikov
 */

#include "QGeoView.h"
#include "QGeoScene.h"
#include "QGeoTiles.h"
#include <core/ssprintf.h>
#include <core/debug.h>

namespace {

static constexpr int wheelAreaMargin = 10;
static const double wheelExponentDown = qPow(2, 1.0 / 5.0);
static const double wheelExponentUp = qPow(2, 1.0 / 2.0);


}

QGeoView::QGeoView(QWidget * parent) :
    Base(parent)
{
  // setContextMenuPolicy(Qt::NoContextMenu);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setOptimizationFlag(DontSavePainterState, true);
  setOptimizationFlag(DontAdjustForAntialiasing, true);
  setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
  setRenderHint(QPainter::Antialiasing, false);
  setCacheMode(QGraphicsView::CacheBackground);
  setMouseTracking(true);
  setBackgroundBrush(QBrush(Qt::darkGray));
  setDragMode(DragMode::NoDrag);

}

QGeoView::QGeoView(QGeoScene * scene, QWidget * parent) :
    Base(scene, parent)
{
}

void QGeoView::setScene(QGeoScene * scene)
{
  Base::setScene(scene);
}

QGeoScene* QGeoView::scene() const
{
  return dynamic_cast<QGeoScene*>(Base::scene());
}

QGeoProjection* QGeoView::projection() const
{
  QGeoScene *scene =
      this->scene();

  return scene ?
      scene->projection() :
      nullptr;
}

QRect QGeoView::viewRect() const
{
  return viewport()->rect();
}

QRectF QGeoView::sceneViewRect() const
{
  return mapToScene(viewport()->rect()).boundingRect();
}

QGeoRect QGeoView::geoViewRect() const
{
  const QGeoProjection *p = this->projection();
  return p ? p->projToGeo(sceneViewRect()) : QGeoRect();
}

QPixmap QGeoView::grabViewPixmap() const
{
  QWidget * w = viewport();
  return w ? w->grab(w->rect()) : QPixmap();
}

QGeoViewCameraState QGeoView::getCameraState() const
{
  return QGeoViewCameraState(const_cast<QGeoView*>(this),
      azimuth_,
      scale_,
      sceneViewRect(),
      state_ == State::Animation);
}

void QGeoView::cameraTo(const QGeoViewCameraActions & actions, bool animation)
{
  const QGeoViewCameraState oldState =
      getCameraState();

  blockCameraUpdate();

  changeState((animation) ?
      State::Animation :
      State::Idle);

  cameraScale(actions.scale());

  cameraMove(actions.projCenter());

  cameraRotate(actions.azimuth());

  unblockCameraUpdate();

  applyCameraUpdate(oldState);
}

void QGeoView::cameraTo(const QGeoRect & georc)
{
  return cameraTo(QGeoViewCameraActions(this).scaleTo(georc));
}

void QGeoView::cameraTo(const QGeoPos & gpos, double latsize, double lonsize)
{
  return cameraTo(QGeoRect(gpos.latitude() - latsize / 2, gpos.longitude() - lonsize / 2,
      gpos.latitude() + latsize / 2, gpos.longitude() + lonsize / 2));
}


void QGeoView::flyTo(const QGeoRect & georc)
{
  if( state_ == State::Idle ) {

    QGeoViewCameraSimpleAnimation *fly =
        new QGeoViewCameraSimpleAnimation(QGeoViewCameraActions(this).scaleTo(georc));

    fly->setDuration(1000);
    fly->start(QAbstractAnimation::DeleteWhenStopped);
  }
}

void QGeoView::flyTo(const QGeoPos & gpos, double latsize, double lonsize)
{
  if( state_ == State::Idle ) {
    QGeoViewCameraSimpleAnimation *fly =
        new QGeoViewCameraSimpleAnimation(QGeoViewCameraActions(this).scaleTo(
            QGeoRect(gpos.latitude() - latsize / 2, gpos.longitude() - lonsize / 2,
                gpos.latitude() + latsize / 2, gpos.longitude() + lonsize / 2)));

    fly->setDuration(1000);
    fly->start(QAbstractAnimation::DeleteWhenStopped);
  }
}

void QGeoView::flyTo(const QGeoViewCameraActions& actions)
{
  if( state_ == State::Idle ) {

    QGeoViewCameraSimpleAnimation *fly =
        new QGeoViewCameraSimpleAnimation(actions);

    fly->setDuration(1000);
    fly->start(QAbstractAnimation::DeleteWhenStopped);
  }
}

double QGeoView::minScale() const
{
  return minScale_;
}

double QGeoView::maxScale() const
{
  return maxScale_;
}

double QGeoView::scale() const
{
  return scale_;
}

double QGeoView::azimuth() const
{
  return azimuth_;
}

Qt::KeyboardModifier QGeoView::itemMouseMoveKeyboardModifiers()
{
  return itemMouseMoveKeyboardModifiers_;
}

void QGeoView::setItemMouseMoveKeyboardModifiers(Qt::KeyboardModifier v)
{
  itemMouseMoveKeyboardModifiers_ = v;
}

void QGeoView::changeState(State newState)
{
  if( newState != state_ ) {

    if( state_ != State::Animation ) {
      state_ = newState;
    }
    else {

      QGeoViewCameraState oldCameraState =
          getCameraState();

      state_ =
          newState;

      applyCameraUpdate(oldCameraState);
    }

    if( state_ == State::Idle ) {
      wheelMouseArea_ = QRect();
      wheelProjAnchor_ = QPointF();
      wheelBestFactor_ = minScale_;
      moveProjAnchor_ = QPointF();
      //rubberBand_.reset();
    }

    Q_EMIT stateChanged(state_);

    //CF_DEBUG("Not call mGeoMap->onMapState(state_)");
    // mGeoMap->onMapState(state_);
  }
}

void QGeoView::blockCameraUpdate()
{
  blockUpdateCount_++;
}

void QGeoView::unblockCameraUpdate()
{
  if( blockUpdateCount_ == 0 ) {
    return;
  }
  blockUpdateCount_--;
}

void QGeoView::applyCameraUpdate(const QGeoViewCameraState & oldState)
{
  if( blockUpdateCount_ < 1 ) {

    QGeoScene *geoScene =
        this->scene();

    if( geoScene ) {

      QGeoViewCameraState newState =
          getCameraState();

      if( oldState != newState ) {

        QList<QGraphicsItem *> items =
            geoScene->items();

        for ( QGraphicsItem * item : items ) {

          QGeoTiles * tiles =
              dynamic_cast<QGeoTiles * >(item);

          if ( tiles && tiles->isVisible() ) {
            tiles->onCamera(oldState, newState);
          }
        }
      }
    }
  }
}

void QGeoView::applyNewCameraState(const QGeoViewCameraState & newState)
{
  if( blockUpdateCount_ < 1 ) {

    QGeoScene *geoScene =
        this->scene();

    if( geoScene ) {

      QGeoViewCameraState newState =
          getCameraState();

      QList<QGraphicsItem*> items =
          geoScene->items();

      for( QGraphicsItem *item : items ) {

        QGeoTiles *tiles =
            dynamic_cast<QGeoTiles*>(item);

        if( tiles && tiles->isVisible() ) {
          tiles->processCamera(newState);
        }
      }
    }
  }
}


void QGeoView::cameraScale(double scale)
{
  const QGeoViewCameraState oldState =
      getCameraState();

  const double oldScale =
      scale_;

  const double newScale =
      qMax(minScale_, qMin(maxScale_, scale));

  if( !qFuzzyCompare(oldScale, newScale) ) {

    const double deltaScale =
        newScale / oldScale;

    Base::scale(deltaScale, deltaScale);

    scale_ =
        newScale;

    applyCameraUpdate(oldState);
  }

}

void QGeoView::cameraRotate(double azimuth)
{
  const QGeoViewCameraState oldState =
      getCameraState();

  const double oldAzimuth =
      fmod(azimuth_, 360);

  const double newAzimuth =
      fmod(azimuth, 360);

  if( !qFuzzyCompare(oldAzimuth, newAzimuth) ) {

    Base::rotate(newAzimuth - oldAzimuth);

    azimuth_ =
        newAzimuth;

    applyCameraUpdate(oldState);
  }

}

void QGeoView::cameraMove(const QPointF & projPos)
{
  const QGeoViewCameraState oldState =
      getCameraState();

  const QPointF oldCenter =
      sceneViewRect().center();

  if( oldCenter != projPos ) {

    Base::centerOn(projPos);

    applyCameraUpdate(oldState);
  }

}

void QGeoView::resizeEvent(QResizeEvent * event)
{
  Base::resizeEvent(event);
  applyNewCameraState(getCameraState());
}

void QGeoView::showEvent(QShowEvent * event)
{
  Base::showEvent(event);
  applyNewCameraState(getCameraState());

  if ( copyImageToClipboardAction_ ) {
    copyImageToClipboardAction_->setEnabled(isVisible());
  }
}

void QGeoView::hideEvent(QHideEvent *event)
{
  Base::hideEvent(event);

  if ( copyImageToClipboardAction_ ) {
    copyImageToClipboardAction_->setEnabled(isVisible());
  }
}

//void QGeoView::paintEvent(QPaintEvent *event)
//{
//  Base::paintEvent(event);
//  Q_EMIT painted();
//}


void QGeoView::contextMenuEvent(QContextMenuEvent *e)
{
  // will be handled later if user will not move ruber band
  //contextMenuEvent_.reset(new QContextMenuEvent(*e));

  contextMenuEvent_.reset(new QContextMenuEvent(
      e->reason(),
      e->pos(),
      e->globalPos(),
      e->modifiers()));

  e->accept();
}

void QGeoView::mousePressEvent(QMouseEvent * event)
{
//  CF_DEBUG("%s: ENTER: isAccepted=%d event: buttons()=0x%0x modifiers()=0x%0x",
//      "QGeoView",
//      event->isAccepted(),
//      event->buttons(),
//      event->modifiers());

  if( event->buttons() == Qt::MouseButton::LeftButton ) {
    if( event->modifiers() == itemMouseMoveKeyboardModifiers_ ) {
      event->ignore();
      Base::mousePressEvent(event);
      //CF_DEBUG("ItemMove: isAccepted()=%d", event->isAccepted());
      if( event->isAccepted() ) {
        return;
      }
    }

    if( event->modifiers() == Qt::NoModifier ) {
      startMoving(event);
      //CF_DEBUG("moveMap: isAccepted()=%d", event->isAccepted());
      if( event->isAccepted() ) {
        return;
      }
    }
  }
  else if( event->buttons() == Qt::MouseButton::RightButton ) {

    event->ignore();
    Base::mousePressEvent(event);
    if ( event->isAccepted() ) {
      // CF_DEBUG("%s: Base::mousePressEvent isAccepted=%d", "QGeoView", event->isAccepted());
      return;
    }

    if( event->modifiers() == Qt::NoModifier ) {
      startSelectionRect(event);
      if ( event->isAccepted() ) {
        // CF_DEBUG("%s: startSelectionRect isAccepted=%d", "QGeoView", event->isAccepted());
        return;
      }
    }
  }


  Base::mousePressEvent(event);


//  CF_DEBUG("%s: LEAVE: isAccepted=%d event: buttons()=0x%0x modifiers()=0x%0x",
//      "QGeoView",
//      event->isAccepted(),
//      event->buttons(),
//      event->modifiers());
}

void QGeoView::mouseReleaseEvent(QMouseEvent * event)
{
//  CF_DEBUG("%s: ENTER: isAccepted=%d event: buttons()=0x%0x modifiers()=0x%0x",
//      "QGeoView",
//      event->isAccepted(),
//      event->buttons(),
//      event->modifiers());

  if( state_ == State::SelectionRect ) {

    if( stopSelectionRect(event) ) {
      contextMenuEvent_.reset();
      return;
    }

    if( contextMenuEvent_ ) {
      Base::contextMenuEvent(contextMenuEvent_.get());
      contextMenuEvent_.reset();
      return;
    }

    return;
  }

  changeState(State::Idle);

  Base::mouseReleaseEvent(event);

//  CF_DEBUG("%s: LEAVE: isAccepted=%d event: buttons()=0x%0x modifiers()=0x%0x",
//      "QGeoView",
//      event->isAccepted(),
//      event->buttons(),
//      event->modifiers());
}

void QGeoView::mouseMoveEvent(QMouseEvent * event)
{
//  if( event->buttons() ) {
//    CF_DEBUG("%s: ENTER: state=%d isAccepted=%d event: buttons()=0x%0x modifiers()=0x%0x",
//        "QGeoView",
//        state_,
//        event->isAccepted(),
//        event->buttons(),
//        event->modifiers());
//  }

  if( state_ == State::Wheel ) {
    mouseMoveForWheel(event);
    // CF_DEBUG("Wheel: isAccepted()=%d", event->isAccepted());
    return;
  }

  if( state_ == State::MovingMap ) {
    moveMap(event);
    // CF_DEBUG("moveMap: isAccepted()=%d", event->isAccepted());
    return;
  }

  if( state_ == State::SelectionRect ) {
    updateSelectionRect(event);
    // CF_DEBUG("SelectionRect: isAccepted()=%d", event->isAccepted());
    return;
  }

  if( event->buttons() == Qt::MouseButton::LeftButton ) {
    if( event->modifiers() == itemMouseMoveKeyboardModifiers_ ) {
      event->ignore();
      Base::mouseMoveEvent(event);
      // CF_DEBUG("ItemMove: isAccepted()=%d", event->isAccepted());
      if( event->isAccepted() ) {
        return;
      }
    }
  }


  // CF_DEBUG("Base::mouseMoveEvent(event)");
  Base::mouseMoveEvent(event);

//  if( event->buttons() ) {
//    CF_DEBUG("%s: LEAVE: isAccepted=%d",
//        "QGeoView",
//        event->isAccepted());
//  }
}

void QGeoView::wheelEvent(QWheelEvent * event)
{
  event->accept();

  if( state_ != State::Wheel ) {

    changeState(State::Wheel);

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)

    QPoint pos(event->position().x(),
        event->position().y());

    wheelMouseArea_ =
        QRect(pos, QSize(1, 1)).
            adjusted(-wheelAreaMargin, -wheelAreaMargin,
            wheelAreaMargin, wheelAreaMargin);

    wheelProjAnchor_ = mapToScene(pos);
#else
    wheelMouseArea_ =
        QRect(event->pos(), QSize(1, 1))
            .adjusted(-wheelAreaMargin, -wheelAreaMargin, wheelAreaMargin, wheelAreaMargin);

    wheelProjAnchor_ = mapToScene(event->pos());
#endif

    wheelBestFactor_ =
        scale_;
  }

  else if( wheelBestFactor_ < scale_ ) {

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)

    wheelProjAnchor_ =
        mapToScene(QPoint(event->position().x(),
            event->position().y()));
#else

    wheelProjAnchor_ =
        mapToScene(event->pos());
#endif

    wheelBestFactor_ = scale_;
  }

  const QGeoViewCameraState oldState =
      getCameraState();

  blockCameraUpdate();

  double newScale =
      scale_;

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)

  if( event->angleDelta().y() > 0 ) {
    newScale *= wheelExponentDown;
  }
  else {
    newScale /= wheelExponentUp;
  }

#else

  if( event->delta() > 0 ) {
    newScale *= wheelExponentDown;
  }
  else {
    newScale /= wheelExponentUp;
  }

#endif

//  CF_DEBUG("SCALE %g->%g", scale_, newScale);

  cameraScale(newScale);

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)

  const QPointF projMouse =
      mapToScene(QPoint(event->position().x(),
          event->position().y()));

#else

  const QPointF projMouse =
      mapToScene(event->pos());

#endif

  const double xDelta =
      (projMouse.x() - wheelProjAnchor_.x());

  const double yDelta =
      (projMouse.y() - wheelProjAnchor_.y());

  if( !qFuzzyIsNull(xDelta) || !qFuzzyIsNull(yDelta) ) {

    cameraMove(sceneViewRect().center() - QPointF(xDelta, yDelta));
  }

  unblockCameraUpdate();
  applyCameraUpdate(oldState);

//  Base::wheelEvent(event);
}

void QGeoView::startMoving(QMouseEvent* event)
{
  //  if (!mMouseActions.testFlag(QGV::MouseAction::Move)) {
  //      changeState(QGV::MapState::Idle);
  //      return;
  //  }
  event->accept();
  changeState(State::MovingMap);
  moveProjAnchor_ = mapToScene(event->pos());
}

void QGeoView::startSelectionRect(QMouseEvent* event)
{
//  if (!mMouseActions.testFlag(QGV::MouseAction::Selection) && !mMouseActions.testFlag(QGV::MouseAction::ZoomRect)) {
//      changeState(QGV::MapState::Idle);
//      return;
//  }
  event->accept();

  changeState(State::SelectionRect);

  rubberBand_.reset(new QGeoViewRubberBand(this));
  rubberBand_->setStartPoint(event->pos());
  rubberBand_->show();

//  mSelectionRect->setStartPoint(event->pos());
//  mSelectionRect->showRect();
}

void QGeoView::updateSelectionRect(QMouseEvent* event)
{
  if ( rubberBand_ ) {
    event->accept();
    rubberBand_->setEndPoint(event->pos());
  }
}

bool QGeoView::stopSelectionRect(QMouseEvent * event)
{
  bool rubberBandProcessed = false;

  event->accept();
  changeState(State::Idle);

  if( rubberBand_ ) {

    QRect rect =
        rubberBand_->geometry();

    rubberBand_.reset();

    if( !rect.isEmpty() ) {

      rubberBandProcessed = true;

      const QRectF newProjRect =
          mapToScene(rect).boundingRect();

      const QGeoViewCameraState oldState =
          getCameraState();

      const QRectF oldProjRect =
          oldState.projRect();

      const double scaleFactor =
          qMin(qAbs(oldProjRect.width() / newProjRect.width()), qAbs(oldProjRect.height() / newProjRect.height()));

      QGeoViewCameraSimpleAnimation *fly =
          new QGeoViewCameraSimpleAnimation(
              QGeoViewCameraActions(this).scaleBy(scaleFactor).moveTo(newProjRect.center()));

      fly->setDuration(1000);
      fly->start(QAbstractAnimation::DeleteWhenStopped);
    }
  }

  return rubberBandProcessed;
}

void QGeoView::mouseMoveForWheel(QMouseEvent* event)
{
//  if (!mMouseActions.testFlag(QGV::MouseAction::ZoomWheel)) {
//      changeState(QGV::MapState::Idle);
//      return;
//  }
  event->accept();
  if (!wheelMouseArea_.contains(event->pos())) {
      changeState(State::Idle);
  }
}


void QGeoView::moveMap(QMouseEvent* event)
{
//  if (!mMouseActions.testFlag(QGV::MouseAction::Move)) {
//      changeState(QGV::MapState::Idle);
//      return;
//  }

  event->accept();

  const QPointF projCenter = sceneViewRect().center();
  const QPointF projMouse = mapToScene(event->pos());
  const double xDelta = (moveProjAnchor_.x() - projMouse.x());
  const double yDelta = (moveProjAnchor_.y() - projMouse.y());
  cameraMove(projCenter + QPointF(xDelta, yDelta));
}

///////////////////////////////////////////////////////////////////////////////////////////

QGeoViewRubberBand::QGeoViewRubberBand(QWidget * parent) :
    Base(QRubberBand::Rectangle, parent)
{
}

void QGeoViewRubberBand::setStartPoint(const QPoint & pos)
{
  startPoint_ = endPoint_ = pos;

  setGeometry(startPoint_.x(), startPoint_.y(),
      endPoint_.x() - startPoint_.x(),
      endPoint_.y() - startPoint_.y());
}

void QGeoViewRubberBand::setEndPoint(const QPoint & pos)
{
  endPoint_ = pos;

  setGeometry(startPoint_.x(), startPoint_.y(),
      endPoint_.x() - startPoint_.x(),
      endPoint_.y() - startPoint_.y());
}

///////////////////////////////////////////////////////////////////////////////////////////

QGeoViewCameraActions::QGeoViewCameraActions(QGeoView * geoView) :
    origin_(geoView->getCameraState())
{


  reset();
}

const QGeoViewCameraState& QGeoViewCameraActions::origin() const
{
  return origin_;
}

QGeoViewCameraActions& QGeoViewCameraActions::rebase(const QGeoViewCameraState & origin)
{
  origin_ = origin;
  return *this;
}

QGeoViewCameraActions& QGeoViewCameraActions::reset()
{
  return reset(origin_);
}

QGeoViewCameraActions& QGeoViewCameraActions::reset(const QGeoViewCameraState & origin)
{
  origin_ = origin;
  scale_ = origin_.scale();
  azimuth_ = origin_.azimuth();
  projCenter_ = origin_.projCenter();
  return *this;
}

QGeoViewCameraActions& QGeoViewCameraActions::scaleBy(double factor)
{
  return scaleTo(origin_.scale() * factor);
}

QGeoViewCameraActions& QGeoViewCameraActions::scaleTo(double scale)
{
  scale_ = scale;
  return *this;
}

QGeoViewCameraActions& QGeoViewCameraActions::scaleTo(const QRectF & projRect)
{
  const QRectF oldProjRect =
      origin_.projRect();

  const double scaleFactor =
      qMin(qAbs(oldProjRect.width() / projRect.width()), qAbs(oldProjRect.height() / projRect.height()));

  scale_ =
      origin_.scale() * scaleFactor;

  projCenter_ =
      projRect.center();

  return *this;
}

QGeoViewCameraActions& QGeoViewCameraActions::scaleTo(const QGeoRect & geoRect)
{
  const QRectF projRect =
      origin_.view()->projection()->geoToProj(geoRect);

  return scaleTo(projRect);
}

QGeoViewCameraActions& QGeoViewCameraActions::rotateBy(double angle)
{
  rotateTo(origin_.azimuth() + angle);
  return *this;
}

QGeoViewCameraActions& QGeoViewCameraActions::rotateTo(double azimuth)
{
  azimuth_ = azimuth;
  return *this;
}

QGeoViewCameraActions& QGeoViewCameraActions::moveTo(const QPointF & projPos)
{
  projCenter_ = projPos;
  return *this;
}

QGeoViewCameraActions& QGeoViewCameraActions::moveTo(const QGeoPos & geoPos)
{
  const QPointF projPos =
      origin_.view()->projection()->geoToProj(geoPos);

  return moveTo(projPos);
}

double QGeoViewCameraActions::scale() const
{
  return scale_;
}

double QGeoViewCameraActions::azimuth() const
{
  return azimuth_;
}

QPointF QGeoViewCameraActions::projCenter() const
{
  return projCenter_;
}

QGeoViewCameraAnimation::QGeoViewCameraAnimation(const QGeoViewCameraActions & actions, QObject * parent) :
    Base(parent),
    mDuration(1000),
    mActions(actions)
{
}

QGeoViewCameraAnimation::~QGeoViewCameraAnimation()
{
  // CF_DEBUG("QGeoViewCameraAnimation destroyed");
}


void QGeoViewCameraAnimation::setDuration(int msecs)
{
  mDuration = msecs;
}

int QGeoViewCameraAnimation::duration() const
{
  return mDuration;
}

QGeoViewCameraActions& QGeoViewCameraAnimation::actions()
{
  return mActions;
}

const QGeoViewCameraActions& QGeoViewCameraAnimation::actions() const
{
  return mActions;
}

void QGeoViewCameraAnimation::onStart()
{
}

void QGeoViewCameraAnimation::onStop()
{
}

double QGeoViewCameraAnimation::interpolateScale(double from, double to, double progress)
{
  if( qFuzzyCompare(from, to) ) {
    return from;
  }

  const double expFrom =
      qLn(from) * M_LOG2E;

  const double expTo =
      qLn(to) * M_LOG2E;

  const double delta =
      expTo - expFrom;

  return qPow(2, expFrom + delta * progress);
}

double QGeoViewCameraAnimation::interpolateAzimuth(double from, double to, double progress)
{
  if( qFuzzyCompare(from, to) ) {
    return from;
  }

  const double delta =
      to - from;

  return from + delta * progress;
}

QPointF QGeoViewCameraAnimation::interpolatePos(QPointF from, QPointF to, double progress)
{
  if( from == to ) {
    return from;
  }

  const QPointF delta =
      to - from;

  return from + delta * progress;
}

void QGeoViewCameraAnimation::updateState(QAbstractAnimation::State newState, QAbstractAnimation::State oldState)
{
  QGeoView *geoView =
      mActions.origin().view();

  if( newState == QAbstractAnimation::Running && oldState == QAbstractAnimation::Stopped ) {

    mActions.rebase(geoView->getCameraState());

    connect(geoView, &QGeoView::stateChanged,
        this, &ThisClass::onStateChanged);

    onStart();
  }

  if( newState == QAbstractAnimation::Stopped && oldState != QAbstractAnimation::Stopped ) {

    disconnect(geoView, nullptr,
        this, nullptr);

    geoView->cameraTo(QGeoViewCameraActions(geoView), false);

    onStop();
  }
}

void QGeoViewCameraAnimation::updateCurrentTime(int currentTime)
{
  double progress =
      static_cast<double>(currentTime) / duration();

  if( direction() == Direction::Backward ) {
    progress = 1.0 - progress;
  }

  QGeoViewCameraActions target(mActions);
  target.reset();
  onProgress(progress, target);

  mActions.origin().view()->cameraTo(target, true);
}

void QGeoViewCameraAnimation::onStateChanged(QGeoView::State state)
{
  if( state != QGeoView::State::Animation ) {
    stop();
  }
}

QGeoViewCameraSimpleAnimation::QGeoViewCameraSimpleAnimation(const QGeoViewCameraActions & actions, QObject * parent) :
    Base(actions, parent),
    mEasing(QEasingCurve::Linear)
{
}

void QGeoViewCameraSimpleAnimation::setEasingCurve(const QEasingCurve & easing)
{
  mEasing = easing;
}

void QGeoViewCameraSimpleAnimation::onProgress(double progress, QGeoViewCameraActions & target)
{
  progress =
      mEasing.valueForProgress(progress);

  target.scaleTo(interpolateScale(
      actions().origin().scale(),
      actions().scale(),
      progress));

  target.rotateTo(interpolateAzimuth(
      actions().origin().azimuth(),
      actions().azimuth(),
      progress));

  target.moveTo(interpolatePos(
      actions().origin().projCenter(),
      actions().projCenter(),
      progress));
}

QGeoViewCameraFlyAnimation::QGeoViewCameraFlyAnimation(const QGeoViewCameraActions & actions, QObject * parent) :
    Base(actions, parent),
    mFlyScale(1)
{
  setDuration(3000);
}

void QGeoViewCameraFlyAnimation::onStart()
{

  const QLineF line(actions().projCenter(),
      actions().origin().projCenter());

  const double scaledDistance0 =
      line.length();

  const double scaledDistance1 =
      scaledDistance0 * actions().origin().scale();

  const double scaledDistance2 =
      scaledDistance0 * actions().scale();

  const double projSpeed0 =
      scaledDistance0 / (duration() / 1000);

  const double projSpeed1 =
      scaledDistance1 / (duration() / 1000);

  const double projSpeed2 =
      scaledDistance2 / (duration() / 1000);

  const double expectedSpeed = 300;

  if( projSpeed1 < expectedSpeed && projSpeed2 < expectedSpeed ) {

    setDuration(static_cast<int>(1000.0 * qMax(scaledDistance1, scaledDistance2) / expectedSpeed));

  }

  mFlyScale =
      qMin(actions().scale(), expectedSpeed / projSpeed0);

  mFlyAnchor =
      interpolatePos(actions().origin().projCenter(),
          actions().projCenter(), 0.5);
}

void QGeoViewCameraFlyAnimation::onProgress(double progress, QGeoViewCameraActions & target)
{
  const auto moveCurve =
      QEasingCurve(QEasingCurve::InQuint);

  const double switchThr = 0.5;

  if( progress <= switchThr ) {

    const double flyInter = progress / switchThr;

    target.scaleTo(interpolateScale(actions().origin().scale(),
        mFlyScale,
        flyInter));

    const double moveInter =
        moveCurve.valueForProgress(flyInter);

    target.moveTo(interpolatePos(actions().origin().projCenter(),
        mFlyAnchor, moveInter));

  }
  else {
    const double flyInter =
        (progress - switchThr) / (1.0 - switchThr);

    target.scaleTo(interpolateScale(mFlyScale,
        actions().scale(),
        flyInter));

    const double moveInter =
        1.0 - moveCurve.valueForProgress(1.0 - flyInter);

    target.moveTo(interpolatePos(mFlyAnchor,
        actions().projCenter(),
        moveInter));
  }

  target.rotateTo(interpolateAzimuth(actions().origin().azimuth(),
      actions().azimuth(), progress));
}


bool QGeoView::popuateContextMenu(const QGraphicsSceneContextMenuEvent * , QMenu & )
{
  return false;
}

QAction* QGeoView::addCopyImageToClipboardAction(const QString & text, const QKeySequence & keysequence)
{
  if( !copyImageToClipboardAction_ ) {

    addAction(copyImageToClipboardAction_ = new QAction(text, this));

    if ( !keysequence.isEmpty() ) {

      connect(copyImageToClipboardActionShortcut_ =
          new QShortcut(keysequence, this, nullptr, nullptr, Qt::WidgetShortcut),
          &QShortcut::activated,
          copyImageToClipboardAction_,
          &QAction::trigger);

    }

    connect(copyImageToClipboardAction_, &QAction::triggered,
        [this]() {
          if ( this->isVisible() ) {
            QApplication::clipboard()->setPixmap(viewport()->
                grab(viewport()->rect()));
          }
        });

  }

  return copyImageToClipboardAction_;
}

