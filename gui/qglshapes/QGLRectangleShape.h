/*
 * QGLRectangleShape.h
 *
 *  Created on: Oct 23, 2024
 *      Author: gandriim
 */

#ifndef __QGLRectangleShape_h__
#define __QGLRectangleShape_h__

#include <gui/qglview/QGLView.h>

/**
 * The coordinates are in SCREEN (not OpenGL) pixels
 * */
class QGLRectangleShape :
    public QGLShape
{
public:
  typedef QGLRectangleShape ThisClass;
  typedef QGLShape Base;

  QGLRectangleShape(QObject * parent = nullptr);

  QGLRectangleShape(const QRect & rc, QObject * parent = nullptr);

  QGLRectangleShape(const QPoint & topLeft, const QPoint & bottomRight, QObject * parent = nullptr);


  void setRect(const QRect & rc)
  {
    _rc = rc;
  }

  void setRect(int aleft, int atop, int awidth, int aheight )
  {
    _rc.setLeft(aleft);
    _rc.setTop(atop);
    _rc.setWidth(awidth);
    _rc.setHeight(aheight);
  }

  const QRect &rect() const
  {
    return _rc;
  }

  QPoint topLeft() const
  {
    return _rc.topLeft();
  }

  QPoint bottomRight() const
  {
    return _rc.bottomRight();
  }

  void setPenColor(const QColor &v)
  {
    _pen.setColor(v);
  }

  QColor penColor() const
  {
    return _pen.color();
  }

  void setPenWidth(int v)
  {
    _pen.setWidth(v);
  }

  int penWidth() const
  {
    return _pen.width();
  }

  void setDepth(double v)
  {
    _depth = v;
  }

  double depth() const
  {
    return _depth;
  }

  void draw(QGLView * glview) override;

protected:
  QRect _rc;
  QPen _pen;
  double _depth = 1;

private:
  void initizalize();
};

#endif /* __QGLRectangleShape_h__ */
