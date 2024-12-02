/*
 * QPointSelectionMode.cc
 *
 *  Created on: Nov 1, 2024
 *      Author: amyznikov
 */

#include "QPointSelectionMode.h"
#include <gui/widgets/style.h>

#define ICON_annotation         ":/gui/icons/annotation.png"

namespace serstacker {


///////////////////////////////////////////////////////////////////////////////////////////////////

QToolButton* createPointSelectionModeToolButton(QWidget *parent)
{
  QToolButton *tb = new QToolButton(parent);
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(getIcon(ICON_annotation));
  tb->setText("Selection");
  tb->setToolTip("Point selection modes");
  //tb->setCheckable(true);
  //tb->setPopupMode(QToolButton::ToolButtonPopupMode::MenuButtonPopup);
  tb->setPopupMode(QToolButton::ToolButtonPopupMode::InstantPopup);

  return tb;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


QPointSelection3DRulerMode::QPointSelection3DRulerMode(QObject *parent) :
    Base(parent)
{
  _rulerLine.setTopLevel(true);
  _rulerLine.setEnabled(false);
  _rulerLine.setVisible(false);
}

const QGLLineShape & QPointSelection3DRulerMode::ruler() const
{
  return _rulerLine;
}

void QPointSelection3DRulerMode::setActive(QInputSourceView* sourceView, bool activate)
{
  if ( activate ) {
    _rulerLine.setEnabled(true);
    _rulerLine.setVisible(false);
    sourceView->cloudView()->addShape(&_rulerLine);
  }
  else {
    _rulerLine.setEnabled(false);
    _rulerLine.setVisible(false);
    sourceView->cloudView()->removeShape(&_rulerLine);
  }

  Base::setActive(sourceView, activate);
}

void QPointSelection3DRulerMode::glMouseEvent(QInputSourceView * sourceView, QEvent::Type eventType, int buttons,
    Qt::KeyboardModifiers keyboardModifiers, const QPointF & mousePos,
    bool objHit, double objX, double objY, double objZ)
{
  //CF_DEBUG("QPointSelection3DRulerMode: currentViewType=%d", sourceView->currentViewType());
  if( eventType == QEvent::MouseButtonRelease ) {
    if( _rulerLine.isVisible() ) {
      _rulerLine.setEnableTooltip(false);
    }
  }
  else {

    if( objHit && buttons == Qt::LeftButton ) {

//      uint64_t pid;
//      if (sourceView->cloudView()->findPointID(objX, objY, objZ, &pid)) {
//        Q_EMIT sourceView->glPointClick(pid, mousePos, mouseEventType,
//            mouseButtons, keyboardModifiers);
//      }

      switch (eventType) {

        case QEvent::MouseButtonPress:
          if( _rulerLine.isEnabled() ) {
            _rulerLine.setStart(objX, objY, objZ);
            _rulerLine.setEnd(objX, objY, objZ);
            sourceView->cloudView()->update();

            Q_EMIT rulerChanged();
          }
          break;

        case QEvent::MouseMove:
          if( _rulerLine.isEnabled() ) {

            _rulerLine.setEnd(objX, objY, objZ);

            if( !_rulerLine.isVisible() ) {
              _rulerLine.setVisible(true);
            }

            _rulerLine.setEnableTooltip(true);

            sourceView->cloudView()->update();

            Q_EMIT rulerChanged();
          }
          break;
      }
    }
  }
}

} // namespace serstacker

