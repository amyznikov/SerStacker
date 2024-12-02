/*
 * QInputPointCloudSourceView.cc
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#include "QPointCloudSourceView.h"
#include <core/debug.h>

namespace serstacker {

QPointCloudSourceView::QPointCloudSourceView(QWidget * parent) :
    Base(parent)
{
//  _line.setVisible(false);
//  _line.setTopLevel(true);
//  Base::addShape(&_line);
}

void QPointCloudSourceView::keyPressEvent(QKeyEvent *e)
{
  if( e->key() == Qt::Key_A && e->modifiers() == Qt::KeyboardModifier::ControlModifier ) {

    setEnableSelection(!enableSelection());

    CF_DEBUG("enableSelection=%d", enableSelection());

    e->ignore();
    return;
  }

  return Base::keyPressEvent(e) ;
}

QString QPointCloudSourceView::statusStringForPoint(int cloud_index, int point_index) const
{
  if ( cloud_index >= 0 && cloud_index < (int) _displayPoints.size() ) {

    const std::vector<cv::Vec3f> & cloud  =
        _displayPoints[cloud_index];

    if ( point_index >= 0 && point_index < (int)cloud.size() ) {

      const cv::Vec3f & v =
          cloud[point_index];

      return qsprintf("c=%d p=%d v={ %+g %+g %+g } dist=%g",
          cloud_index, point_index,
          v[0], v[1], v[2],
          cv::norm(v));
    }

  }

  return "";
}


void QPointCloudSourceView::glMouseEvent(QEvent::Type eventType, int keyOrMouseButtons,
    Qt::KeyboardModifiers keyboardModifiers, const QPointF & mousePos,
    bool objHit, double objX, double objY, double objZ)
{
  Q_EMIT glPointSelectionMouseEvent(eventType, keyOrMouseButtons,
      keyboardModifiers, mousePos,
      objHit, objX, objY, objZ);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
} /* namespace serstacker */
