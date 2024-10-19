/*
 * QGLLineShape.h
 *
 *  Created on: Oct 19, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGLLineShape_h__
#define __QGLLineShape_h__

#include <gui/qglview/QGLView.h>

class QGLLineShape :
    public QGLShape
{
public:
  typedef QGLLineShape ThisClass;
  typedef QGLShape Base;

  QGLLineShape(QObject * parent = nullptr);

  QGLLineShape(const QVector3D & start, const QVector3D & end,
      QObject * parent = nullptr);

  void setStart(const QVector3D &v);
  const QVector3D &start() const;

  void setEnd(const QVector3D & v);
  const QVector3D & end() const;

  void setVisible(bool v) override;
  void draw(QGLView * glview) override;

protected:
  QVector3D _start;
  QVector3D _end;
};

#endif /* __QGLLineShape_h__ */
