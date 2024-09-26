/*
 * QInputPointCloudSourceView.cc
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#include "QPointCloudSourceView.h"
#include <core/debug.h>

namespace serstacker {

static inline double distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
}

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

void QPointCloudSourceView::keyReleaseEvent(QKeyEvent *e)
{
  return Base::keyReleaseEvent(e) ;
}

void QPointCloudSourceView::glSelectionEvent(const QPointF & click_pos, double objX, double objY, double objZ)
{
  if ( !_displayPoints.empty() ) {

    int best_cloud_index = -1;
    int best_point_index = -1;
    double best_distance = DBL_MAX;

    for ( int cloud_index = 0, num_clouds = _displayPoints.size(); cloud_index < num_clouds; ++ cloud_index ) {

      const std::vector<cv::Vec3f> & cloud  =
          _displayPoints[cloud_index];

      for ( int point_index = 0, num_points = cloud.size(); point_index < num_points; ++ point_index ) {

        const cv::Vec3f & p =
            cloud[point_index];

        const double d =
            distance(p[0], p[1], p[2], objX, objY, objZ);

        if ( d < best_distance ) {
          best_distance = d;
          best_cloud_index = cloud_index;
          best_point_index = point_index;
        }

      }

    }


    if ( best_point_index >= 0 ) {

      CF_DEBUG("click: x=%g y=%g obj: X=%g Y=%g Z=%g cloud:%d point:%d distance=%g",
          click_pos.x(), click_pos.y(), objX, objY, objZ,
          best_cloud_index, best_point_index, best_distance);

    }

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
} /* namespace serstacker */
