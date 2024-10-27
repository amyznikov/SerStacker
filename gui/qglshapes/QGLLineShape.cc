/*
 * QGLLineShape.cc
 *
 *  Created on: Oct 19, 2024
 *      Author: amyznikov
 */

#include "QGLLineShape.h"
#include <gui/widgets/qsprintf.h>
#include <core/debug.h>

QGLLineShape::QGLLineShape(QObject * parent) :
  Base(parent)
{
}

QGLLineShape::QGLLineShape(const QVector3D & start, const QVector3D & end, QObject * parent) :
  Base(parent),
  _start(start),
  _end(end)
{
}

void QGLLineShape::setStart(double objX, double objY, double objZ)
{
  setStart(QVector3D(objX, objY, objZ));
}

void QGLLineShape::setStart(const QVector3D &v)
{
  _start = v;
}

const QVector3D &QGLLineShape::start() const
{
  return _start;
}

void QGLLineShape::setEnd(double objX, double objY, double objZ)
{
  setEnd(QVector3D(objX, objY, objZ));
}

void QGLLineShape::setEnd(const QVector3D & v)
{
  _end = v;
}

const QVector3D & QGLLineShape::end() const
{
  return _end;
}

void QGLLineShape::setVisible(bool v)
{
  Base::setVisible(v);
  QToolTip::hideText();
}

void QGLLineShape::draw(QGLView *glview)
{
  QToolTip::showText(glview->mapToGlobal(QPoint(24, 24)),
      qsprintf("<strong>Line:</strong><br>\n"
          "start = (%+g %+g %+g)<br>"
          "end = (%+g %+g %+g)<br>"
          "<strong>Length</strong> = %g (%+g %+g %+g)",
          _start.x(), _start.y(), _start.z(),
          _end.x(), _end.y(), _end.z(),
          QVector3D(_end - _start).length(),
          _end.x()-_start.x(), _end.y()-_start.y(),_end.z()-_start.z()));

  glLineWidth(3);
  glColor3ub(255, 255, 0);

  glBegin(GL_LINES);

  glColor3ub(255, 255, 0);
  glVertex3f(_start.x(), _start.y(), _start.z());
  glVertex3f(_end.x(), _end.y(), _end.z());

  glEnd();
}

