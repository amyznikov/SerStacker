/*
 * QTestGLView.cc
 *
 *  Created on: Feb 1, 2023
 *      Author: amyznikov
 */

#include "QTestGLView.h"
#include <core/get_time.h>
#include <core/debug.h>

namespace qgltest {

QTestGLView::QTestGLView(QWidget * parent) :
    Base(parent)
{
}

double QTestGLView::eyeX() const
{
  return eye_.x();
}

void QTestGLView::setEyeX(double value)
{
  eye_.setX(value);
  dirty_ = true;
  update();
}

double QTestGLView::eyeY() const
{
  return eye_.y();
}

void QTestGLView::setEyeY(double value)
{
  eye_.setY(value);
  dirty_ = true;
  update();
}

double QTestGLView::eyeZ() const
{
  return eye_.z();
}

void QTestGLView::setEyeZ(double value)
{
  eye_.setZ(value);
  dirty_ = true;
  update();
}

void QTestGLView::glInit()
{
  Base::glInit();
}

void QTestGLView::glPreDraw()
{
  Base::glPreDraw();
}

void QTestGLView::glPostDraw()
{
  Base::glPostDraw();
}

void QTestGLView::glCleanup()
{
  Base::glCleanup();
}

void QTestGLView::glDraw()
{
  glColor3ub(200, 200, 200);
  drawMainAxes();
  //drawArrow(QVector3D(0, 0, 0), QVector3D(40, 0, 0), 0.2, 8);


  glPointSize(10);

  glBegin(GL_POINTS);

    glColor3ub(255, 255, 255);
    glVertex3f(0,  0,  0);

    glColor3ub(255, 0, 0);
    glVertex3f(10,  0,  0);

    glColor3ub(0, 255, 0);
    glVertex3f(0,  10,  0);

    glColor3ub(0, 0, 255);
    glVertex3f(0,  0,  10);

    glColor3ub(255, 255, 0);
    glVertex3f(target_.x(),  target_.y(),  target_.z());

  glEnd(/*GL_POINTS*/);

  glLineWidth(2);

  glBegin(GL_LINES);

    glColor3ub(255, 0,  0);
    glVertex3f(-20,  0,  0);
    glVertex3f(+20,  0,  0);

    glColor3ub(0, 255,  0);
    glVertex3f(0,  -20,  0);
    glVertex3f(0,  +20,  0);

    glColor3ub(0, 0,  255);
    glVertex3f(0,  0,  -20);
    glVertex3f(0,  0,  +20);

  glEnd(/*GL_LINES*/);

  static QFont font("Monospace", 12);
  glColor3ub(255, 255, 0);

  printf(20, 20, font, "time is %g ms", get_realtime_ms());

  printf(12, 0, 0, font, "X");

}


} /* namespace qgltest */
