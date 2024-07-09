/*
 * c_draw_ellipsoid_routine.cc
 *
 *  Created on: Jul 5, 2024
 *      Author: amyznikov
 */

#include "c_draw_ellipsoid_routine.h"
#include <core/proc/pose.h>
#include <core/proc/ellipsoid_bbox.h>

static bool project_ellipsoid_point(const double lat, const double lon,
    const double A, const double B, const double C,
    const cv::Matx33d & R,
    const cv::Point2f & center,
    cv::Point2f * pos)
{
  const cv::Vec3d v1 =
      R * cv::Vec3d(A * cos(lat) * cos(lon),
          B * cos(lat) * sin(lon),
          C * sin(lat));

  pos->x = v1(0) + center.x;
  pos->y = v1(1) + center.y;

//  const double d =
//      v1.dot(cv::Vec3d(0, 0, 1));

  return true;
}


void c_draw_ellipsoid_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, equatorial_radius1, "");
  BIND_PCTRL(ctls, equatorial_radius2, "");
  BIND_PCTRL(ctls, polar_radius, "");
  BIND_PCTRL(ctls, center, "");
  BIND_PCTRL(ctls, orientation, "");
  BIND_PCTRL(ctls, longitude_step, "");
  BIND_PCTRL(ctls, latidute_step, "");
  BIND_PCTRL(ctls, outline_color, "");
  BIND_PCTRL(ctls, lines_color, "");

}

bool c_draw_ellipsoid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, equatorial_radius1);
    SERIALIZE_PROPERTY(settings, save, *this, equatorial_radius2);
    SERIALIZE_PROPERTY(settings, save, *this, polar_radius);
    SERIALIZE_PROPERTY(settings, save, *this, center);
    SERIALIZE_PROPERTY(settings, save, *this, orientation);
    SERIALIZE_PROPERTY(settings, save, *this, longitude_step);
    SERIALIZE_PROPERTY(settings, save, *this, latidute_step);
    SERIALIZE_PROPERTY(settings, save, *this, outline_color);
    SERIALIZE_PROPERTY(settings, save, *this, lines_color);
    return true;
  }
  return false;
}



bool c_draw_ellipsoid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  /*
   * (x/A)^2 + (y/B)^2 + (z/C)^2 = 1;
   *
   */

  const cv::Size image_size =
      image.size();

  const cv::Vec3d & rotation =
      orientation() * CV_PI / 180;

  const cv::Matx33d R =
      build_rotation2(rotation);


  const double A =
      equatorial_radius1();

  const double B =
      equatorial_radius2();

  const double C =
      polar_radius();

  const cv::Point2f center =
      center_.x >= 0 && center_.y >= 0 ? center_ :
          cv::Point2f(image_size.width / 2, image_size.height / 2);

  const cv::RotatedRect bbox =
      ellipsoid_bbox(center, A, B, C, R.t());



  if( latidute_step_ > 0 ) {

    const double lon_step =
        8 / std::max(bbox.size.width, bbox.size.height);

    const double lat_step =
        latidute_step_ * CV_PI / 180;

    cv::Point2f cpos, ppos;

    double lat, lon;

    for( lat = 0; lat < CV_PI / 2; lat += lat_step ) {

      project_ellipsoid_point(-lat, lon = 0, A, B, C, R, center, &ppos);
      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        project_ellipsoid_point(-lat, lon, A, B, C, R, center, &cpos);
        cv::line(image, ppos, cpos, lines_color_, 1, cv::LINE_AA);
      }

      if ( lat == 0 ) {
        continue;
      }

      project_ellipsoid_point(lat, lon = 0, A, B, C, R, center, &ppos);
      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        project_ellipsoid_point(lat, lon, A, B, C, R, center, &cpos);
        cv::line(image, ppos, cpos, lines_color_, 1, cv::LINE_AA);
      }
    }
  }

  if( longitude_step_ > 0 ) {

    const double lon_step =
        longitude_step_ * CV_PI / 180;

    const double lat_step =
        8 / std::max(bbox.size.width, bbox.size.height);

    cv::Point2f cpos, ppos;
    double lat, lon;

    for( double lon = 0; lon < 2 * CV_PI; lon += lon_step ) {

      project_ellipsoid_point(lat = -CV_PI / 2, lon, A, B, C, R, center, &ppos);

      for( lat = -CV_PI / 2 + lat_step; lat <= CV_PI / 2; lat += lat_step, ppos = cpos ) {
        project_ellipsoid_point(lat, lon, A, B, C, R, center, &cpos);
        cv::line(image, ppos, cpos, lines_color_, 1, cv::LINE_AA);
      }
    }

  }

  cv::ellipse(image, bbox, outline_color_, 1, cv::LINE_AA);

  return true;
}



