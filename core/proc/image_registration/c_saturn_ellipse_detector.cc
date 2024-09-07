/*
 * c_saturn_ellipse_detector.cc
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#include "c_saturn_ellipse_detector.h"
#include <core/proc/ellipsoid.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/pose.h>
#include <core/debug.h>

bool c_saturn_ellipse_detector::detect(cv::InputArray image, cv::InputArray mask)
{
  static constexpr double radius_ratio =
      54400. / 60300.;

  const cv::Size image_size =
      image.size();


  _pose =
      _options.pose * CV_PI / 180;

  double & A = _axes(0);
  double & B = _axes(1);
  double & C = _axes(2);

  if ( !_options.auto_location ) {

    A = B = _options.equatorial_radius ;
    C = _options.equatorial_radius * radius_ratio;
    _ring_radius = _options.ring_radius;

    _center = _options.center.x >= 0 && _options.center.y >= 0 ? _options.center :
        cv::Point2f(image_size.width / 2, image_size.height / 2);

    _saturn_detected = true;

  }
  else {

    _saturn_detected =
        detect_saturn(image, _options.se_close_radius,
            _saturn_bounding_box,
            _saturn_mask);

    if ( !_saturn_detected ) {
      CF_ERROR("detect_saturn() fails");
      return false;
    }

    C = _saturn_bounding_box.size.height / 2;
    A = B = C / radius_ratio;
    _ring_radius = _saturn_bounding_box.size.width / 2;
    _center = _saturn_bounding_box.center;
    _pose(1) = _saturn_bounding_box.angle * CV_PI / 180;

    if ( _options.draw.print_debug_info ) {

      CF_DEBUG("\nDetected Saturn ellipse:\n"
          "equatorial_radius=%g [px]\n"
          "ring_radius=%g [px]\n"
          "center= ( %g; %g ) [px]\n"
          "pose= ( %g; %g ; %g) [deg]\n"
          "\n"
          "\n",
          A,
          _ring_radius,
          _center.x, _center.y,
          _pose(0) * 180 / CV_PI, _pose(1) * 180 / CV_PI, _pose(2) * 180 / CV_PI
      );
    }

  }

  _planetary_disk_ellipse =
      ellipsoid_bbox(_center, A, B, C,
          build_rotation2(_pose).t());

  _planetary_disk_ellipse_mask.create(image.size());
  _planetary_disk_ellipse_mask.setTo(0);

  draw_ellipse(_planetary_disk_ellipse_mask,
      _planetary_disk_ellipse,
      cv::Scalar::all(255), -1,
      cv::LINE_8);

  if ( _options.auto_location ) {

  }

  return true;
}


bool c_saturn_ellipse_detector::draw_detected(cv::InputOutputArray image) const
{
  static constexpr double radius_ratio =
      54400. / 60300.;

  const cv::Size image_size =
      image.size();

  const cv::RotatedRect & sbox =
      _saturn_bounding_box;

  cv::Mat smask;

  const double & A = _axes(0);
  const double & B = _axes(1);
  const double & C = _axes(2);

  if ( _options.draw.deltat  != 0 ) {

    cv::Mat2f rmap;
    cv::Mat1b rmask;
    cv::Mat remapped_image;

    compute_saturn_zrotation_deltat_remap(image.size(), _center,
        _axes, _pose, _options.draw.deltat ,
        rmap, rmask);

    //    compute_ellipsoid_zrotation_remap(image.size(), center,
    //        cv::Vec3d(A, B, C), rotation, _options.draw.zrotation * CV_PI / 180,
    //        rmap, rmask);

    cv::remap(image.getMat(), remapped_image, rmap, cv::noArray(), cv::INTER_LINEAR,
        cv::BORDER_CONSTANT, cv::Scalar::all(0));

    remapped_image.copyTo(image, rmask);
  }

  const cv::Matx33d R =
      build_rotation2(_pose);

  const cv::RotatedRect & bbox =
      _planetary_disk_ellipse;
      //ellipsoid_bbox(center, A, B, C, R.t());

  if( _options.draw.latidute_step > 0 ) {

    const double lon_step =
        8 / std::max(bbox.size.width, bbox.size.height);

    const double lat_step =
        _options.draw.latidute_step * CV_PI / 180;

    cv::Point2d cpos, ppos;

    double lat, lon;

    for( lat = 0; lat < CV_PI / 2; lat += lat_step ) {

      ellipsoid_to_cart2d(-lat, lon = 0, A, B, C, R, _center, &ppos);

      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if( ellipsoid_to_cart2d(-lat, lon, A, B, C, R, _center, &cpos) ) {
          cv::line(image, ppos, cpos, _options.draw.line_color, 1, cv::LINE_AA);
        }
      }

      if ( lat == 0 ) {
        continue;
      }

      ellipsoid_to_cart2d(lat, lon = 0, A, B, C, R, _center, &ppos);

      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if ( ellipsoid_to_cart2d(lat, lon, A, B, C, R, _center, &cpos) ) {
          cv::line(image, ppos, cpos, _options.draw.line_color , 1, cv::LINE_AA);
        }
      }

    }
  }

  if( _options.draw.longitude_step > 0 ) {

    const double lon_step =
        _options.draw.longitude_step * CV_PI / 180;

    const double lat_step =
        8 / std::max(bbox.size.width, bbox.size.height);

    cv::Point2d cpos, ppos;
    double lat, lon;

    for( double lon = 0; lon < 2 * CV_PI; lon += lon_step ) {

      ellipsoid_to_cart2d(lat = -CV_PI / 2, lon, A, B, C, R, _center, &ppos);

      for( lat = -CV_PI / 2 + lat_step; lat <= CV_PI / 2; lat += lat_step, ppos = cpos ) {
        if( ellipsoid_to_cart2d(lat, lon, A, B, C, R, _center, &cpos) ) {
          cv::line(image, ppos, cpos, _options.draw.line_color, 1, cv::LINE_AA);
        }
      }
    }

  }

  //cv::ellipse(image, bbox, outline_color_, 1, cv::LINE_AA);
  draw_ellipse(image, bbox, _options.draw.outline_color, 1, cv::LINE_AA);

  if ( _options.draw.show_ring && _ring_radius > 0 ) {

    const cv::RotatedRect ring_bbox =
        rotated_ellipse_bbox(_center, _ring_radius, _ring_radius, R.t());

    // cv::ellipse(image, ring_bbox, 0.5*outline_color_, 5, cv::LINE_AA);
    draw_ellipse(image, ring_bbox, _options.draw.outline_color, 1, cv::LINE_AA);
  }

  if( !_options.auto_location && _options.draw.show_sbox && _saturn_detected ) {
    draw_rotated_rect(image, sbox, _options.draw.outline_color, 1, cv::LINE_AA);
  }

  return true;
}

