/*
 * QTestGLView.h
 *
 *  Created on: Feb 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QTestGLView_h__
#define __QTestGLView_h__

#include <gui/qglview/QGLView.h>

namespace qgltest {

class QTestGLView:
    public QGLView
{
  Q_OBJECT;
public:
  typedef QTestGLView ThisClass;
  typedef QGLView Base;

  QTestGLView(QWidget * parent = nullptr);

  double eyeX() const;
  void setEyeX(double value);

  double eyeY() const;
  void setEyeY(double value);

  double eyeZ() const;
  void setEyeZ(double value);

protected:
  void glInit() override;
  void glPreDraw() override;
  void glDraw() override;
  void glPostDraw() override;
  void glCleanup() override;

protected:
};

} /* namespace qgltest */

#endif /* __QTestGLView_h__ */
