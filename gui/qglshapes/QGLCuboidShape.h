/*
 * QGLCuboidShape.h
 *
 *  Created on: Oct 20, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGLCuboidShape_h__
#define __QGLCuboidShape_h__

#include <gui/qglview/QGLView.h>

class QGLCuboidShape :
    public QGLShape
{
public:
  typedef QGLCuboidShape ThisClass;
  typedef QGLShape Base;

  QGLCuboidShape(QObject * parent = nullptr);

  void setVisible(bool v) override;
  void draw(QGLView * glview) override;

protected:
};

#endif /* __QGLCuboidShape_h__ */
