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

  static QGraphicsView* getActiveView(const QGraphicsSceneEvent * event);
  static QPainterPath shapeFromPath(const QPainterPath &path, const QPen & pen);

  void setUpdatingPos(bool v);
  bool inUpdatingPos() const;

Q_SIGNALS:
  void itemChanged(QGraphicsShape* _this);
  void populateContextMenuReuested(const QGraphicsSceneContextMenuEvent * event, QMenu * menu);

protected:
  Q_DISABLE_COPY(QGraphicsShape);
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = nullptr) override;
  void contextMenuEvent(QGraphicsSceneContextMenuEvent * event) override;
  virtual bool popuateContextMenu(const QGraphicsSceneContextMenuEvent * event, QMenu & menu);

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


//
//class QGraphicsLineShape :
//    public QGraphicsShape,
//    public QGraphicsLineItem
//{
//  Q_OBJECT;
//public:
//  typedef QGraphicsLineShape ThisClass;
//  typedef QGraphicsLineItem Base;
//
//  explicit QGraphicsLineShape(QGraphicsItem *parent = nullptr);
//  explicit QGraphicsLineShape(const QLineF &line, QGraphicsItem *parent = nullptr);
//  explicit QGraphicsLineShape(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem *parent = nullptr);
//
//  QPainterPath shape() const override;
//  QRectF boundingRect() const override;
//  void populateContextMenu(QMenu & menu, const QGraphicsSceneContextMenuEvent & e) override;
//
//protected:
//  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
//  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
//  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
//  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
//
//protected:
//  enum line_moving_mode {
//    not_moving = 0,
//    move_p1,
//    move_p2,
//    move_whole_line,
//  } line_moving_mode_ = not_moving;
//
//  bool p1Locked_ = false;
//  bool p2Locked_ = false;
//};
//
//class QGraphicsRectShape :
//    public QGraphicsShape,
//    public QGraphicsRectItem
//{
//  Q_OBJECT;
//public:
//  typedef QGraphicsRectShape ThisClass;
//  typedef QGraphicsRectItem Base;
//
//  explicit QGraphicsRectShape(QGraphicsItem *parent = nullptr);
//  explicit QGraphicsRectShape(const QRectF &rect, QGraphicsItem *parent = nullptr);
//  explicit QGraphicsRectShape(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent = nullptr);
//
//  QPainterPath shape() const override;
//  QRectF boundingRect() const override;
//
//protected:
//  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
//  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
//  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
//  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
//
//protected:
//  enum rect_moving_mode {
//    not_moving = 0,
//    move_lt,
//    move_rt,
//    move_rb,
//    move_lb,
//    move_whole_rect,
//  } rect_moving_mode_ = not_moving;
//};


#endif /* __QGraphicsShape_h__ */
