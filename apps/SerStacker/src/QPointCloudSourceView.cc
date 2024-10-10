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


void QPointCloudSourceView::glPointSelection(double objX, double objY, double objZ,
    const QPointF & mousePos,
    QEvent::Type mouseEventType,
    Qt::MouseButtons mouseButtons,
    Qt::KeyboardModifiers modifiers)
{
  if ( !_displayPoints.empty() ) {


    int best_cloud_index = -1;
    int best_point_index = -1;
    double best_delta = DBL_MAX;

    static const auto distance =
        [](double x1, double y1, double z1, double x2, double y2, double z2) -> double {
          return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
        };


    for ( int cloud_index = 0, num_clouds = _displayPoints.size(); cloud_index < num_clouds; ++ cloud_index ) {

      const std::vector<cv::Vec3f> & cloud  =
          _displayPoints[cloud_index];

      for ( int point_index = 0, num_points = cloud.size(); point_index < num_points; ++ point_index ) {

        const cv::Vec3f & p =
            cloud[point_index];

        const double d =
            distance(p[0], p[1], p[2], objX, objY, objZ);

        if ( d < best_delta ) {
          best_delta = d;
          best_cloud_index = cloud_index;
          best_point_index = point_index;
        }

      }

    }

    if( best_point_index >= 0 ) {

//      const cv::Vec3f & p =
//          _displayPoints[best_cloud_index][best_point_index];
//
//      const double pointDistance =
//          cv::norm(p);
//
//      CF_DEBUG("click: c:%d p:%d X=%g Y=%g Z=%g Distance=%g delta=%g",
//          best_cloud_index, best_point_index,
//          p[0], p[1], p[2], pointDistance,
//          best_delta);

      Q_EMIT pointClicked(best_cloud_index, best_point_index);
    }

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
} /* namespace serstacker */
