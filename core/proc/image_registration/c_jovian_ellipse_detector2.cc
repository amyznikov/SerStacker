/*
 * c_jovian_ellipse_detector2.cc
 *
 *  Created on: Aug 23, 2024
 *      Author: amyznikov
 */

#include "c_jovian_ellipse_detector2.h"
#include <core/proc/ellipsoid.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/morphology.h>
#include <core/proc/fitEllipseLM.h>
#include <core/proc/pose.h>
#include <core/debug.h>

static constexpr double jovian_axis_ratio =
    0.93512560845968779724;

bool c_jovian_ellipse_detector2::detect(cv::InputArray image, cv::InputArray mask)
{
  const cv::Size image_size =
      image.size();

  double & A = _axes(0);
  double & B = _axes(1);
  double & C = _axes(2);

  _pose = _options.pose * CV_PI / 180;
  _reference_image_size = image_size;

  if ( !_options.auto_location ) {

    A = B = _options.equatorial_radius ;
    C = _options.equatorial_radius * jovian_axis_ratio;

    _center = _options.center.x >= 0 && _options.center.y >= 0 ? _options.center :
        cv::Point2f(image_size.width / 2, image_size.height / 2);

    _center += _options.offset;

    _detected = true;

  }
  else {

    std::vector<cv::Point2f> component_edge_points;
    cv::Rect detected_component_rect;
    cv::RotatedRect ellipse;
    cv::Mat gray_image;

    /////////////////////////////////////////////////////////////////////////////////////////
    // Convert input image to gray scale

    if( image.channels() == 1 ) {
      image.getMat().copyTo(gray_image);
    }
    else {
      cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    }

    /////////////////////////////////////////////////////////////////////////////////////////
    // Detect planetary disk mask and use fitEllipseAMS() on planetary disk edge
    // as initial ellipse estimate

    bool fOk =
        simple_planetary_disk_detector(gray_image, mask,
            _options.gbsigma,
            _options.stdev_factor,
            _options.se_close_radius,
            nullptr,
            &detected_component_rect,
            &_planetary_disk_ellipse_mask,
            nullptr,
            nullptr);
    //        detect_planetary_disk(gray_image, mask,
    //            1,
    //            _options.stdev_factor,
    //            &_planetary_disk_ellipse_mask,
    //            &detected_component_rect);

    if( !fOk ) {
      CF_ERROR("detect_planetary_disk() fails");
      return false;
    }

    morphological_gradient(_planetary_disk_ellipse_mask,
        _detected_planetary_disk_edge,
        cv::Mat1b(3, 3, 255),
        cv::BORDER_CONSTANT);

    cv::findNonZero(_detected_planetary_disk_edge,
        component_edge_points);

    // Use PCA on image gradients to compute jovian disk orientation
    ellipse.angle =
        compute_jovian_orientation_pca(gray_image,
            _planetary_disk_ellipse_mask,
            detected_component_rect);

    if( ellipse.angle > 90 ) {
      ellipse.angle -= 180;
    }
    else if( ellipse.angle < -90 ) {
      ellipse.angle += 180;
    }

    fitEllipseLM1(component_edge_points,
        jovian_axis_ratio, // b / a
        ellipse.angle * CV_PI / 180, // radians
        &_planetary_disk_ellipse);

//    CF_DEBUG("\n"
//        "fitEllipseLM1: width=%g height=%g angle=%g",
//        _planetary_disk_ellipse.size.width,
//        _planetary_disk_ellipse.size.height,
//        _planetary_disk_ellipse.angle);

    _planetary_disk_ellipse.center +=
        _options.offset;

    //final_planetary_disk_ellipse_ = ellipse;


    // create also filed binary ellipse mask
    _final_planetary_disk_mask.create(image.size());
    _final_planetary_disk_mask.setTo(0);
    cv::ellipse(_final_planetary_disk_mask, _planetary_disk_ellipse, 255, -1, cv::LINE_8);
    cv::erode(_final_planetary_disk_mask, _final_planetary_disk_mask, cv::Mat1b(3, 3, 255));


    A = B = _planetary_disk_ellipse.size.width / 2;
    C = _planetary_disk_ellipse.size.width * jovian_axis_ratio / 2;
    _center = _planetary_disk_ellipse.center;
    _pose(1) = _planetary_disk_ellipse.angle * CV_PI / 180;

    if ( _options.draw.print_debug_info ) {

      CF_DEBUG("\nDetected Jovian ellipse:\n"
          "equatorial_radius=%g [px]\n"
          "center= ( %g; %g ) [px]\n"
          "pose= ( %g; %g ; %g) [deg]\n"
          "\n"
          "\n",
          A,
          _center.x - _options.offset.x, _center.y - _options.offset.y,
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


  CF_DEBUG("center=(%g %g) axes=(%g %g %g) pose=(%+g %+g %+g)",
      _center.x, _center.y,
      _axes(0), _axes(1), _axes(2),
      _pose(0) * 180 / CV_PI, _pose(1) * 180 / CV_PI, _pose(2) * 180 / CV_PI);

  if ( _options.auto_location ) {

  }

  return true;
}


bool c_jovian_ellipse_detector2::draw_detected(cv::InputOutputArray image)
{
  const cv::Size image_size =
      image.size();

  const cv::RotatedRect & sbox =
      _planetary_disk_ellipse;

  cv::Mat smask;

  const double & A = _axes(0);
  const double & B = _axes(1);
  const double & C = _axes(2);

//  double A, B, C, ring_radius;
//  cv::Point2f center;
//  cv::Vec3d rotation;

//  const cv::Vec3d & orientation =
//      _options.orientation;
//
//  if ( _saturn_detected ) {
//    C = sbox.size.height / 2;
//    A = B = C / radius_ratio;
//    ring_radius = sbox.size.width / 2;
//    center = sbox.center;
//    rotation = cv::Vec3d(orientation(0), sbox.angle, orientation(2)) * CV_PI / 180;
//  }
//  else {
//    A = B = _options.equatorial_radius ;
//    C = A * radius_ratio;
//    ring_radius = _options.ring_radius;
//    center = _options.center.x >= 0 && _options.center.y >= 0 ? _options.center : cv::Point2f(image_size.width / 2, image_size.height / 2);
//    rotation = orientation * CV_PI / 180;
//  }

  if ( _options.draw.deltat  != 0 || _options.draw.show_bmask || _options.draw.show_wmask ) {

    cv::Mat remapped_image;
    compute_derotation_for_time(_options.draw.deltat);

    cv::remap(image.getMat(), remapped_image, _current_remap, cv::noArray(), cv::INTER_LINEAR,
        cv::BORDER_CONSTANT, cv::Scalar::all(0));

    remapped_image.copyTo(image, _current_bmask);
  }

  const cv::Matx33d R =
      build_rotation2(_pose);

//  const cv::RotatedRect & bbox =
//      _planetary_disk_ellipse;
      //ellipsoid_bbox(center, A, B, C, R.t());

  if( _options.draw.latidute_step > 0 ) {

    const double lon_step =
        8 / std::max(sbox.size.width, sbox.size.height);

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
        8 / std::max(sbox.size.width, sbox.size.height);

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
  draw_ellipse(image, sbox, _options.draw.outline_color, 1, cv::LINE_AA);


  if( !_options.auto_location && _options.draw.show_sbox && _detected ) {
    draw_rotated_rect(image, sbox, _options.draw.outline_color, 1, cv::LINE_AA);
  }

  return true;
}


double c_jovian_ellipse_detector2::compute_jovian_orientation_pca(const cv::Mat & gray_image,
    const cv::Mat & planetary_disk_mask, const cv::Rect & component_rect)
{
  static float kernel[] = { +1. / 12, -2. / 3, +0., +2. / 3, -1. / 12 };
  static const cv::Matx<float, 1, 5> Kx = cv::Matx<float, 1, 5>(kernel); // / 1.5;
  static const cv::Matx<float, 5, 1> Ky = cv::Matx<float, 5, 1>(kernel); // / 1.5;

  const cv::Size size = component_rect.size();

  cv::Mat g;

  if( _options.pca_blur <= 0 ) {
    g = gray_image(component_rect);
  }
  else {
    cv::GaussianBlur(gray_image(component_rect), g, cv::Size(-1, -1),
        _options.pca_blur, _options.pca_blur,
        cv::BORDER_REPLICATE);
  }

  cv::filter2D(g, _pca_gx, CV_32F, Kx, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(g, _pca_gy, CV_32F, Ky, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  const int erode_size = std::max(3, 2 * (int) (0.075 *
      std::min(size.width, size.height)) + 1);

  cv::Mat1b mask;

  cv::erode(planetary_disk_mask(component_rect), mask,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erode_size, erode_size)),
      cv::Point(-1, -1),
      1,
      cv::BORDER_CONSTANT,
      cv::Scalar::all(0));

  _pca_gx.setTo(0, ~mask);
  _pca_gy.setTo(0, ~mask);

  const int npts =
      cv::countNonZero(mask);

  cv::Mat1f data_pts(npts, 2);

  int ii = 0;
  for( int y = 0; y < mask.rows; ++y ) {
    for( int x = 0; x < mask.cols; ++x ) {
      if( mask[y][x] ) {
        data_pts[ii][0] = _pca_gx[y][x];
        data_pts[ii][1] = _pca_gy[y][x];
        ++ii;
      }
    }
  }

  cv::PCA pca(data_pts, cv::noArray(),
      cv::PCA::DATA_AS_ROW);

  // Select second eigen vector for ellipse orientation
  const cv::Vec2f eigen_vec(pca.eigenvectors.at<float>(1, 0),
      pca.eigenvectors.at<float>(1, 1));

  const double angle =
      atan2(eigen_vec[1],
          eigen_vec[0]);

  return angle * 180 / CV_PI;
}

bool c_jovian_ellipse_detector2::compute_derotation_for_angle(double zrotation_deg)
{
//      compute_ellipsoid_zrotation_deltat_remap(image.size(), _center,
//          _axes, _pose, _options.draw.deltat ,
//          rmap, rmask);
//
      //    compute_ellipsoid_zrotation_remap(image.size(), center,
      //        cv::Vec3d(A, B, C), rotation, _options.draw.zrotation * CV_PI / 180,
      //        rmap, rmask);

  const double & A =
      _axes(0);

  const double & B =
      _axes(1);

  const double & C =
      _axes(2);

  CF_DEBUG("zrotation_deg=%g center=(%g %g) axes=(%g %g %g) pose=(%+g %+g %+g)",
      zrotation_deg,
      _center.x, _center.y,
      _axes(0), _axes(1), _axes(2),
      _pose(0) * 180 / CV_PI, _pose(1) * 180 / CV_PI, _pose(2) * 180 / CV_PI);

  _planetary_disk_ellipse =
      ellipsoid_bbox(_center, A, B, C,
          build_rotation2(_pose).t());

  _planetary_disk_ellipse_mask.create(_reference_image_size);
  _planetary_disk_ellipse_mask.setTo(0);

  draw_ellipse(_planetary_disk_ellipse_mask,
      _planetary_disk_ellipse,
      cv::Scalar::all(255), -1,
      cv::LINE_8);

  compute_ellipsoid_zrotation_remap(_reference_image_size,
      _center,
      _axes,
      _pose,
      zrotation_deg * CV_PI / 180,
      _current_remap,
      _current_bmask);


  _current_bmask =
      cv::Mat1b(_current_remap.size(),
          (uint8_t)(0));

  draw_ellipse(_current_bmask,
      _planetary_disk_ellipse,
      cv::Scalar::all(255),
      -1,
      cv::LINE_8);

  //  _current_bmask.convertTo(_current_wmask,
  //      CV_32F);

  cv::distanceTransform(_current_bmask, _current_wmask,
      cv::DIST_L2, cv::DIST_MASK_PRECISE,
      _current_wmask.depth() );

  cv::remap(_current_wmask, _current_wmask,
      _current_remap, cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  double min, max;
  cv::minMaxLoc(_current_wmask, &min, &max);
  cv::multiply(_current_wmask, 1. / max, _current_wmask);

  return true;
}

bool c_jovian_ellipse_detector2::compute_derotation_for_time(double deltat_sec)
{
  // Jupiter daily rotation period is 9h 55m 30s.

  static constexpr double rotation_period_sec =
      9. * 3660 + 55. * 60 + 30.;

  const double rotation_angle_deg =
      360 * deltat_sec / rotation_period_sec;

  return compute_derotation_for_angle(rotation_angle_deg);
}
