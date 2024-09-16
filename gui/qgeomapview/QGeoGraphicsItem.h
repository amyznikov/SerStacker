/*
 * QGeoGraphicsItem.h
 *
 *  Created on: Nov 26, 2022
 *      Author: amyznikov
 *
 *  // To Find the active view for item's mouse event:
 *  QGraphicsView *view = nullptr;
 *  if (event->widget())
 *    view = qobject_cast<QGraphicsView *>(event->widget()->parentWidget());
 */

#pragma once
#ifndef __QGeoGraphicsItem_h__
#define __QGeoGraphicsItem_h__

#include "QGeoScene.h"

class QGeoViewCameraState;

class QGeoGraphicsItem:
    public QGraphicsObject
{
  Q_OBJECT;
  Q_PROPERTY(QString name READ name WRITE setName)
  Q_PROPERTY(QString description READ description WRITE setDescription)

public:
  typedef QGeoGraphicsItem ThisClass;
  typedef QGraphicsObject Base;

  QGeoGraphicsItem(QGraphicsItem *parent = nullptr);
  QGeoGraphicsItem(const QString & name, const QString & description, QGraphicsItem *parent = nullptr);
  ~QGeoGraphicsItem();
  Q_DISABLE_COPY(QGeoGraphicsItem);

  void setName(const QString& name);
  const QString & name() const;

  void setDescription(const QString& description);
  const QString & description() const;

  QGeoScene * geoScene() const;
  QGeoProjection * projection() const;

  void setRenderHints(QPainter::RenderHints hints, bool on = true);
  QPainter::RenderHints renderHintsOn() const;
  QPainter::RenderHints renderHintsOff() const;

  static QGraphicsView* getActiveView(const QGraphicsSceneEvent * event);

  void setUpdatingPos(bool newValue);
  bool inUpdatingPos() const;

  virtual void onCamera(const QGeoViewCameraState& oldState,
      const QGeoViewCameraState & newState);

  virtual bool populateContextMenu(const QGraphicsSceneContextMenuEvent * event,
      QMenu & menu);

Q_SIGNALS:
  void geoPosChanged(const QGeoPos & pos);
  void populateContextMenuReuested(const QGraphicsSceneContextMenuEvent * event, QMenu * menu);

protected:
  virtual void updateProjected(const QGeoScene* geoScene = nullptr);
  virtual void updateGeo(const QGeoScene* geoScene = nullptr);
  virtual void onGeoPosChanged(const QGeoPos & pos);

protected: // QGraphicsObject
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  void contextMenuEvent(QGraphicsSceneContextMenuEvent * event) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

protected:
  QString name_;
  QString description_;
  QPainter::RenderHints renderHintsOn_;
  QPainter::RenderHints renderHintsOff_;
  QGraphicsScene * myPreviousScene_ = nullptr;
  int inUpdatingPos_ = 0;

  // See https://stackoverflow.com/questions/40590798/how-to-keep-the-size-and-position-of-qgraphicsitem-when-scaling-the-view
  // Assume that the object anchor is always at item's coordinate (0,0)
  struct QIgnoreTransformationStub:
      public QGraphicsItem
  {
    QIgnoreTransformationStub()
    {
      setFlag(ItemHasNoContents, true);
    }
    QRectF boundingRect(void) const override
    { // empty
      return QRectF();
    }
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override
    { // empty
    }
  } * ignoreTransformation_ = nullptr;
};

#endif /* __QGeoGraphicsItem_h__ */
