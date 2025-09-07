/*
 * QGraphicsShape.h
 *
 *  Created on: Oct 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsShape_h__
#define __QGraphicsShape_h__

#include <QtWidgets/QtWidgets>

class QGraphicsShape :
    public QGraphicsObject
{
  Q_OBJECT;
  Q_PROPERTY(QString name READ name WRITE setName)
  Q_PROPERTY(QString description READ description WRITE setDescription)
public:
  typedef QGraphicsShape ThisClass;
  typedef QGraphicsObject Base;

  QGraphicsShape(QGraphicsItem *parent = nullptr);
  QGraphicsShape(const QString & name, const QString & description, QGraphicsItem *parent = nullptr);
  ~QGraphicsShape();

  void setName(const QString& name);
  const QString & name() const;

  void setDescription(const QString& description);
  const QString & description() const;

  void setRenderHints(QPainter::RenderHints hints, bool on = true);
  QPainter::RenderHints renderHintsOn() const;
  QPainter::RenderHints renderHintsOff() const;

  void setSnapToPixelGrid(bool v);
  bool snapToPixelGrid() const;

  static QGraphicsView* getActiveView(const QGraphicsSceneEvent * event);
  static QPainterPath shapeFromPath(const QPainterPath &path, const QPen & pen);

  void setUpdatingPos(bool v);
  bool inUpdatingPos() const;

  static void load(QGraphicsShape * shape, const QSettings & settings,
      const QString & sectionName);

  static void save(const QGraphicsShape * shape, QSettings & settings,
      const QString & sectionName);

  static QGraphicsShape * load(const QSettings & settings,
      const QString & sectionName);

  virtual void popuateContextMenu(QMenu & menu, const QPoint & viewpos);

Q_SIGNALS:
  void itemChanged(QGraphicsShape* _this);
  void populateContextMenuReuested(QGraphicsShape* _this, QMenu & menu, const QPoint & viewpos);

public:  // frequently used utilities
  static double distance(const QPointF & p1, const QPointF & p2);
  static double distance(const QPoint & p1, const QPoint & p2);
  static double distance_from_point_to_line(const QPointF & p, const QPointF & lp1, const QPointF & lp2);
  static double distance_from_point_to_line(const QPoint & p, const QPoint & lp1, const QPoint & lp2);
  static double distance_from_point_to_line(const QPointF & p, const QLineF & line);

protected:
  Q_DISABLE_COPY(QGraphicsShape);
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  virtual void updateGeometry();
  virtual void onSceneChange();
  virtual void onSceneHasChanged();

protected:
  QString name_;
  QString description_;
  QPainter::RenderHints renderHintsOn_;
  QPainter::RenderHints renderHintsOff_;
  QGraphicsScene * myPreviousScene_ = nullptr;
  bool snapToPixelGrid_ = false;
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


#endif /* __QGraphicsShape_h__ */
