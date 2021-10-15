/*
 * QGraphicsItems.h
 *
 *  Created on: Oct 12, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsItems_h__
#define __QGraphicsItems_h__

#include <QtWidgets/QtWidgets>

class QGraphicsArrowItem
    : public QGraphicsLineItem
{
public:
  enum {
    Type = UserType + 4
  };

  QGraphicsArrowItem(QGraphicsItem *parent = Q_NULLPTR);

  int type() const override
  {
    return Type;
  }

  QRectF boundingRect() const override;
  QPainterPath shape() const override;

  void setColor(const QColor &color)
  {
    myColor = color;
  }

  void updatePosition();

protected:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0) override;

private:
  QColor myColor;
  QPolygonF arrowHead;
};



#endif /* __QGraphicsItems_h__ */
