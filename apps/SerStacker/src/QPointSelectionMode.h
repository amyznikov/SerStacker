/*
 * QPointSelectionMode.h
 *
 *  Created on: Nov 1, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPointSelectionMode_h_
#define __QPointSelectionMode_h_

#include "QInputSourceView.h"

namespace serstacker {

class QPointSelection3DRulerMode :
    public QPointSelectionMode
{
  Q_OBJECT;
public:
  typedef QPointSelection3DRulerMode ThisClass;
  typedef QPointSelectionMode Base;

  QPointSelection3DRulerMode(QObject * parent = nullptr);

  const QGLLineShape & ruler() const;

  void setActive(QInputSourceView* sourceView, bool activate) final;

  void glMouseEvent(QInputSourceView * sourceView, const QPointF &mousePos, QEvent::Type mouseEventType,
      Qt::MouseButtons mouseButtons, Qt::KeyboardModifiers keyboardModifiers,
      bool objHit, double objX, double objY, double objZ) final;


Q_SIGNALS:
  void rulerChanged();

protected:
  QGLLineShape _rulerLine;
};



QToolButton* createPointSelectionModeToolButton(QWidget * parent);

} // namespace serstacker

#endif /* __QPointSelectionMode_h_ */
