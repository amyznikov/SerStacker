/*
 * QGeoView.h
 *
 *  Created on: Nov 25, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGeoView___h__
#define __QGeoView___h__

#include <QtWidgets/QtWidgets>
#include "QGeoViewCameraState.h"
#include "QGeoPos.h"
#include "QGeoRect.h"

class QGeoScene;
class QGeoProjection;
class QGeoViewCameraActions;
class QGeoViewRubberBand;


class QGeoView:
    public QGraphicsView
{
  Q_OBJECT;
public:
  typedef QGeoView ThisClass;
  typedef QGraphicsView Base;

  enum class State {
      Idle,
      Animation,
      Wheel,
      MovingMap,
      SelectionRect,
  };

  QGeoView(QWidget *parent = nullptr);
  QGeoView(QGeoScene * scene, QWidget *parent = nullptr);

  void setScene(QGeoScene *scene);
  QGeoScene * scene() const;

  QGeoProjection* projection() const;

  QGeoViewCameraState getCameraState() const;

  QRect viewRect() const;
  QRectF sceneViewRect() const;
  QGeoRect geoViewRect() const;
  QPixmap grabViewPixmap() const;

  double minScale() const;
  double maxScale() const;
  double scale() const;
  double azimuth() const;


  void cameraTo(const QGeoRect & georc);
  void cameraTo(const QGeoPos & gpos, double latsize, double lonsize);
  void cameraTo(const QGeoViewCameraActions& actions, bool animation = false);

  void flyTo(const QGeoRect & georc);
  void flyTo(const QGeoPos & gpos, double latsize, double lonsize);
  void flyTo(const QGeoViewCameraActions& actions);

  void setItemMouseMoveKeyboardModifiers(Qt::KeyboardModifier v);
  Qt::KeyboardModifier itemMouseMoveKeyboardModifiers();

  QAction * addCopyImageToClipboardAction(const QString & text,
      const QKeySequence & keysequence = QKeySequence());

  virtual bool popuateContextMenu(const QGraphicsSceneContextMenuEvent * event,
      QMenu & menu);

Q_SIGNALS:
  void stateChanged(State newState);
  //  void painted();

protected:
  //  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent *event) override;
  void contextMenuEvent(QContextMenuEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
#if QT_CONFIG(wheelevent)
  void wheelEvent(QWheelEvent *event) override;
#endif

protected:
  void changeState(State newState);
  void blockCameraUpdate();
  void unblockCameraUpdate();
  void applyCameraUpdate(const QGeoViewCameraState & oldState);
  void applyNewCameraState(const QGeoViewCameraState & newState);
  void cameraScale(double scale);
  void cameraRotate(double azimuth);
  void cameraMove(const QPointF& projPos);
  void startMoving(QMouseEvent* event);
  void startSelectionRect(QMouseEvent* event);
  bool stopSelectionRect(QMouseEvent* event);
  void updateSelectionRect(QMouseEvent* event);
  void mouseMoveForWheel(QMouseEvent* event);
  void moveMap(QMouseEvent* event);

protected:
  State state_ = State::Idle;
  int blockUpdateCount_ = 0;
  double minScale_ = 1e-8;
  double maxScale_ = 1e+2;
  double scale_ = 1;
  double azimuth_ = 0;
  QRect wheelMouseArea_;
  QPointF wheelProjAnchor_;
  double wheelBestFactor_ = 1;
  QPointF moveProjAnchor_;
  QScopedPointer<QGeoViewRubberBand> rubberBand_;
  QScopedPointer<QContextMenuEvent> contextMenuEvent_;

  Qt::KeyboardModifier itemMouseMoveKeyboardModifiers_ = Qt::AltModifier;
  QAction * copyImageToClipboardAction_ = nullptr;
  QShortcut * copyImageToClipboardActionShortcut_  = nullptr;

};


class QGeoViewRubberBand :
    public QRubberBand
{
public:
  typedef QGeoViewRubberBand ThisClass;
  typedef QRubberBand Base;

  QGeoViewRubberBand(QWidget * parent = nullptr);

  void setStartPoint(const QPoint & pos);
  void setEndPoint(const QPoint & pos);

protected:
  QPoint startPoint_;
  QPoint endPoint_;
};



class QGeoViewCameraActions
{
public:
  typedef QGeoViewCameraActions ThisClass;

  explicit QGeoViewCameraActions(QGeoView * geoView);

  const QGeoViewCameraState & origin() const;

  QGeoViewCameraActions& rebase(const QGeoViewCameraState & origin);
  QGeoViewCameraActions& reset();
  QGeoViewCameraActions& reset(const QGeoViewCameraState & origin);
  QGeoViewCameraActions& scaleBy(double factor);
  QGeoViewCameraActions& scaleTo(double scale);
  QGeoViewCameraActions& scaleTo(const QRectF & projRect);
  QGeoViewCameraActions& scaleTo(const QGeoRect & geoRect);
  QGeoViewCameraActions& rotateBy(double angle);
  QGeoViewCameraActions& rotateTo(double azimuth);
  QGeoViewCameraActions& moveTo(const QPointF & projPos);
  QGeoViewCameraActions& moveTo(const QGeoPos & geoPos);

  double scale() const;
  double azimuth() const;
  QPointF projCenter() const;

protected:
  QGeoViewCameraState origin_;
  double scale_;
  double azimuth_;
  QPointF projCenter_;
};

class QGeoViewCameraAnimation:
    public QAbstractAnimation
{
public:
  typedef QGeoViewCameraAnimation ThisClass;
  typedef QAbstractAnimation Base;

  QGeoViewCameraAnimation(const QGeoViewCameraActions & actions, QObject * parent = nullptr);
  ~QGeoViewCameraAnimation();

  void setDuration(int msecs);
  int duration() const override;
  QGeoViewCameraActions & actions();
  const QGeoViewCameraActions & actions() const;

protected:
  virtual void onStart();
  virtual void onStop();
  virtual void onProgress(double progress, QGeoViewCameraActions & target) = 0;

  static double interpolateScale(double from, double to, double progress);
  static double interpolateAzimuth(double from, double to, double progress);
  static QPointF interpolatePos(QPointF from, QPointF to, double progress);

private:
  void updateState(QAbstractAnimation::State newState, QAbstractAnimation::State oldState) override;
  void updateCurrentTime(int currentTime) override;
  void onStateChanged(QGeoView::State state);

private:
  int mDuration;
  QGeoViewCameraActions mActions;
};

class QGeoViewCameraSimpleAnimation:
    public QGeoViewCameraAnimation
{
public:
  typedef QGeoViewCameraSimpleAnimation ThisClass;
  typedef QGeoViewCameraAnimation Base;

  QGeoViewCameraSimpleAnimation(const QGeoViewCameraActions & actions,
      QObject * parent = nullptr);

  void setEasingCurve(const QEasingCurve & easing);

private:
  void onProgress(double progress, QGeoViewCameraActions & target) override;

private:
  QEasingCurve mEasing;
};

class QGeoViewCameraFlyAnimation:
    public QGeoViewCameraAnimation
{
public:
  typedef QGeoViewCameraFlyAnimation ThisClass;
  typedef QGeoViewCameraAnimation Base;

  QGeoViewCameraFlyAnimation(const QGeoViewCameraActions & actions,
      QObject * parent = nullptr);

private:
  void onStart() override;
  void onProgress(double progress, QGeoViewCameraActions & target) override;

private:
  double mFlyScale;
  QPointF mFlyAnchor;
};

#endif /* __QGeoView___h__ */
