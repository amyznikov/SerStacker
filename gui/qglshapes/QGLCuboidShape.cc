/*
 * QGLCuboidShape.cc
 *
 *  Created on: Oct 20, 2024
 *      Author: amyznikov
 */

#include "QGLCuboidShape.h"

QGLCuboidShape::QGLCuboidShape(QObject * parent) :
  Base(parent)
{
}

void QGLCuboidShape::setVisible(bool v)
{
  Base::setVisible(v);
}

void QGLCuboidShape::draw(QGLView * glview)
{
}
