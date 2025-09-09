/*
 * QGLRectangleShape.cc
 *
 *  Created on: Oct 23, 2024
 *      Author: gandriim
 */

#include "QGLRectangleShape.h"
#include <core/debug.h>

QGLRectangleShape::QGLRectangleShape(QObject * parent) :
  Base(parent)
{
  initizalize();
}

QGLRectangleShape::QGLRectangleShape(const QRect & rc, QObject * parent) :
  Base(parent),
  _rc(rc)
{
  initizalize();
}

QGLRectangleShape::QGLRectangleShape(const QPoint & topLeft, const QPoint & bottomRight, QObject * parent) :
    Base(parent),
    _rc(topLeft, bottomRight)
{
  initizalize();
}

void QGLRectangleShape::initizalize()
{
  _pen.setColor(Qt::red);
  _pen.setWidth(3);
  _pen.setCosmetic(true);
}


void QGLRectangleShape::draw(QGLView * glview)
{
  if ( !_rc.isEmpty() ) {

    QPainter painter(glview);

    painter.setPen(_pen);
    painter.drawRect(_rc);
  }
}
















