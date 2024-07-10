/*
 * c_draw_saturn_ellipse_routine.cc
 *
 *  Created on: Jul 10, 2024
 *      Author: amyznikov
 */

#include "c_draw_saturn_ellipse_routine.h"
#include <core/proc/pose.h>
#include <core/proc/ellipsoid.h>




static bool project_point(const double lat, const double lon,
    const double A, const double B, const double C,
    const cv::Matx33d & R,
    const cv::Point2f & center,
    cv::Point2f * pos)
{

  const cv::Vec3d v0(A * cos(lat) * cos(lon),
      B * cos(lat) * sin(lon),
      C * sin(lat));

  const cv::Vec3d v1 =
      R * v0;

  pos->x = v1(0) + center.x;
  pos->y = v1(1) + center.y;

  // check point visibility based on rotated surface normal direction
  const double zr =
      R(2, 0) * v0(0) / (A * A) +
      R(2, 1) * v0(1) / (B * B) +
      R(2, 2) * v0(2) / (C * C);

    return zr >= 0;
}


void c_draw_saturn_ellipse_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, equatorial_radius, "");
  BIND_PCTRL(ctls, ring_radius, "");
  BIND_PCTRL(ctls, center, "");
  BIND_PCTRL(ctls, orientation, "");
  BIND_PCTRL(ctls, longitude_step, "");
  BIND_PCTRL(ctls, latidute_step, "");
  BIND_PCTRL(ctls, outline_color, "");
  BIND_PCTRL(ctls, lines_color, "");
}

bool c_draw_saturn_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, equatorial_radius);
    SERIALIZE_PROPERTY(settings, save, *this, ring_radius);
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

bool c_draw_saturn_ellipse_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  static constexpr double radius_ratio =
      54400. / 60300.;

  const cv::Size image_size =
      image.size();

  const cv::Vec3d & rotation =
      orientation() * CV_PI / 180;

  const cv::Matx33d R =
      build_rotation2(rotation);

  const double A =
      equatorial_radius_ ;

  const double B =
      equatorial_radius_;

  const double C =
      A * radius_ratio;

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

      project_point(-lat, lon = 0, A, B, C, R, center, &ppos);
      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if( project_point(-lat, lon, A, B, C, R, center, &cpos) ) {
          cv::line(image, ppos, cpos, lines_color_, 1, cv::LINE_AA);
        }
      }

      if ( lat == 0 ) {
        continue;
      }

      project_point(lat, lon = 0, A, B, C, R, center, &ppos);
      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if ( project_point(lat, lon, A, B, C, R, center, &cpos) ) {
          cv::line(image, ppos, cpos, lines_color_ , 1, cv::LINE_AA);
        }
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

      project_point(lat = -CV_PI / 2, lon, A, B, C, R, center, &ppos);

      for( lat = -CV_PI / 2 + lat_step; lat <= CV_PI / 2; lat += lat_step, ppos = cpos ) {
        if( project_point(lat, lon, A, B, C, R, center, &cpos) ) {
          cv::line(image, ppos, cpos, lines_color_, 1, cv::LINE_AA);
        }
      }
    }

  }

  //cv::ellipse(image, bbox, outline_color_, 1, cv::LINE_AA);
  draw_ellipse(image, bbox, outline_color_, 1, cv::LINE_AA);


  if ( ring_radius_ > 0 ) {

    const cv::RotatedRect ring_bbox =
        rotated_ellipse_bbox(center, ring_radius_, ring_radius_, R.t());

    // cv::ellipse(image, ring_bbox, 0.5*outline_color_, 5, cv::LINE_AA);
    draw_ellipse(image, ring_bbox, outline_color_, 1, cv::LINE_AA);

  }

  return true;
}
