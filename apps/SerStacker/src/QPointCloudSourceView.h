/*
 * QInputPointCloudSourceView.h
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputPointCloudSourceView_h__
#define __QInputPointCloudSourceView_h__

#include <gui/qglview/QGLPointCloudView.h>
#include <gui/qglshapes/QGLLineShape.h>

namespace serstacker {

class QPointCloudSourceView :
    public QGLPointCloudView
{
  Q_OBJECT;
public:
  typedef QPointCloudSourceView ThisClass;
  typedef QGLPointCloudView Base;

  QPointCloudSourceView(QWidget * parent = nullptr);

  QString statusStringForPoint(int cloud_index, int point_index) const;

Q_SIGNALS:
  void pointClicked(int cloud_index, int point_index);

protected:
  void keyPressEvent(QKeyEvent *event) final;
  void glPointSelection(double objX, double objY, double objZ,
      const QPointF & mousePos,
      QEvent::Type mouseEventType,
      Qt::MouseButtons mouseButtons,
      Qt::KeyboardModifiers modifiers) final;

protected:
  QGLLineShape _line;
};

/////////////////
} /* namespace serstacker */

#endif /* __QInputPointCloudSourceView_h__ */
