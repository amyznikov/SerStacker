/*
 * c_draw_saturn_ellipse_routine.cc
 *
 *  Created on: Jul 10, 2024
 *      Author: amyznikov
 */

#include "c_draw_saturn_ellipse_routine.h"
#include <core/proc/pose.h>
#include <core/proc/ellipsoid.h>
#include <core/proc/planetary-disk-detection.h>


static bool detect_saturn(cv::InputArray _image, cv::RotatedRect & output_bbox,
    cv::OutputArray output_mask = cv::noArray())
{

  cv::Point2f centrold;
  cv::Point2f geometrical_center;
  double gbsigma = 1;
  double stdev_factor = 0.5;

  cv::Mat1b component_mask, rotated_component_mask;
  cv::Mat pts;

  bool fOk =
      simple_planetary_disk_detector(_image,
          cv::noArray(),
          &centrold,
          gbsigma,
          stdev_factor,
          nullptr,
          &component_mask,
          &geometrical_center);

  if ( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }

  cv::findNonZero(component_mask, pts);
  pts = pts.reshape(1, pts.rows);
  pts.convertTo(pts, CV_32F);

  cv::PCA pca(pts, cv::noArray(), cv::PCA::DATA_AS_ROW);

  const cv::Mat1f mean = pca.mean;
  const cv::Point2f center(mean(0, 0), mean(0, 1));
  const cv::Matx22f eigenvectors = pca.eigenvectors;

  CF_DEBUG("cntr=(x=%g y=%g)\n"
      " eigenvectors= {\n"
      "  %+20g %+20g\n"
      "  %+20g %+20g\n"
      "}\n"
      "\n",
      center.x, center.y,
      eigenvectors(0, 0), eigenvectors(0, 1),
      eigenvectors(1, 0), eigenvectors(1, 1));

  const double angle =
      atan2(eigenvectors(0, 1), eigenvectors(0, 0)) * 180 / CV_PI;

  const cv::Matx23f M =
      getRotationMatrix2D(center, angle, 1);

  cv::warpAffine(component_mask, rotated_component_mask, M,
      component_mask.size(),
      cv::INTER_NEAREST,
      cv::BORDER_CONSTANT);

  const cv::Rect rc =
      cv::boundingRect(rotated_component_mask);

  output_bbox.center = center;
  output_bbox.size = rc.size();
  output_bbox.angle = angle;

  if ( output_mask.needed() ) {
    output_mask.move(component_mask);
  }

  return true;
}


static void draw_rotated_rect(cv::InputOutputArray _img, const cv::RotatedRect & rc, const cv::Scalar & color,
    int thickness, int line_type)
{
  // We take the edges that OpenCV calculated for us
  cv::Point2f vertices2f[4];
  rc.points(vertices2f);

  // Convert them so we can use them in a fillConvexPoly
  cv::Point vertices[4];
  for( int i = 0; i < 4; ++i ) {
    vertices[i].x = cvRound(vertices2f[i].x);
    vertices[i].y = cvRound(vertices2f[i].y);
  }

  if( thickness < 0 ) {
    fillConvexPoly(_img, vertices, 4, color, line_type);
  }
  else {
    const cv::Point * ppts[] = { vertices };
    const int npts[] = { 4 };
    cv::polylines(_img, ppts, npts, 1, true, color, thickness, line_type, 0);
  }
}

void c_draw_saturn_ellipse_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, auto_location, "");

  BIND_PCTRL(ctls, equatorial_radius, "");
  BIND_PCTRL(ctls, ring_radius, "");
  BIND_PCTRL(ctls, center, "");
  BIND_PCTRL(ctls, orientation, "");
  BIND_PCTRL(ctls, longitude_step, "");
  BIND_PCTRL(ctls, latidute_step, "");
  BIND_PCTRL(ctls, outline_color, "");
  BIND_PCTRL(ctls, lines_color, "");

  BIND_PCTRL(ctls, show_smask, "");

}

bool c_draw_saturn_ellipse_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, auto_location);
    SERIALIZE_PROPERTY(settings, save, *this, equatorial_radius);
    SERIALIZE_PROPERTY(settings, save, *this, ring_radius);
    SERIALIZE_PROPERTY(settings, save, *this, center);
    SERIALIZE_PROPERTY(settings, save, *this, orientation);
    SERIALIZE_PROPERTY(settings, save, *this, longitude_step);
    SERIALIZE_PROPERTY(settings, save, *this, latidute_step);
    SERIALIZE_PROPERTY(settings, save, *this, outline_color);
    SERIALIZE_PROPERTY(settings, save, *this, lines_color);
    SERIALIZE_PROPERTY(settings, save, *this, show_smask);
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

  cv::RotatedRect sbbox;
  cv::Mat smask;
  bool saturn_detected = false;


  if( auto_location_ ) {
    saturn_detected =
        detect_saturn(image, sbbox,
            smask);

    if ( show_smask_ && saturn_detected ) {
      smask.copyTo(image);
      return true;
    }

  }


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

    cv::Point2d cpos, ppos;

    double lat, lon;

    for( lat = 0; lat < CV_PI / 2; lat += lat_step ) {

      ellipsoid_to_cart2d(-lat, lon = 0, A, B, C, R, center, &ppos);
      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if( ellipsoid_to_cart2d(-lat, lon, A, B, C, R, center, &cpos) ) {
          cv::line(image, ppos, cpos, lines_color_, 1, cv::LINE_AA);
        }
      }

      if ( lat == 0 ) {
        continue;
      }

      ellipsoid_to_cart2d(lat, lon = 0, A, B, C, R, center, &ppos);
      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if ( ellipsoid_to_cart2d(lat, lon, A, B, C, R, center, &cpos) ) {
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

    cv::Point2d cpos, ppos;
    double lat, lon;

    for( double lon = 0; lon < 2 * CV_PI; lon += lon_step ) {

      ellipsoid_to_cart2d(lat = -CV_PI / 2, lon, A, B, C, R, center, &ppos);

      for( lat = -CV_PI / 2 + lat_step; lat <= CV_PI / 2; lat += lat_step, ppos = cpos ) {
        if( ellipsoid_to_cart2d(lat, lon, A, B, C, R, center, &cpos) ) {
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

  if ( saturn_detected ) {
    draw_rotated_rect(image, sbbox, outline_color_, 1, cv::LINE_AA);
  }


  return true;
}
