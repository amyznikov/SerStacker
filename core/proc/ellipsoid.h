/*
 * ellipsoid.h
 *
 *  Created on: Jul 9, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __ellipsoid_h__
#define __ellipsoid_h__

#include <opencv2/opencv.hpp>


/**
 * Given 3D ellipsoid with sem-axes A, B, C and rotated by matrix R
 * compute its outline (shadow) bounding box, appropriate for drawing
 * with cv::ellipse()
 *
 * https://math.stackexchange.com/questions/573055/projection-of-ellipsoid
 * https://www.r-5.org/files/books/computers/algo-list/image-processing/vision/Richard_Hartley_Andrew_Zisserman-Multiple_View_Geometry_in_Computer_Vision-EN.pdf
 * Hartley & Zisserman’s Multiple View Geometry In Computer Vision. Result 8.9 on page 201
 *
 * Example :
 *   const cv::Size image_size = image.size();
 *   const cv::Vec3d & rotation = orientation() * CV_PI / 180;
 *   const cv::Matx33d R = build_rotation2(rotation);
 *   const double A = equatorial_radius1();
 *   const double B = equatorial_radius2();
 *   const double C = polar_radius();
 *   const cv::Point2f center = cv::Point2f(image_size.width / 2, image_size.height / 2);
 *   const cv::RotatedRect bbox = ellipsoid_bbox(center, A, B, C, R.t());
 *   cv::ellipse(image, bbox, cv::Scalar::all(255), 1, cv::LINE_AA);
 */
cv::RotatedRect ellipsoid_bbox(const cv::Point2f & center,
    double A, double B, double C,
    const cv::Matx33d & R);

/**
 * Compute rotated 2D ellipse outline bound box,
 * appropriate to draw Saturn rings with cv::ellipse()
 * */
cv::RotatedRect rotated_ellipse_bbox(const cv::Point2f & center, double A, double B,
    const cv::Matx33d & R);

/**
 * Replacement for cv::ellipsePoly() with better angular precision
 * */
void ellipse_poly(const cv::Point2f & center,
    const cv::Size2f & axes,
    double angle,
    std::vector<cv::Point> & pts);

/*
 * Replacement for cv::ellipse() with better angular precision
 * */
void draw_ellipse(cv::InputOutputArray _img, const cv::RotatedRect & rc,
    const cv::Scalar & color, int thickness, int line_type);

/*
 * Draw cv::RotatedRect
 * */
void draw_rotated_rect(cv::InputOutputArray _img, const cv::RotatedRect & rc,
    const cv::Scalar & color, int thickness, int line_type = cv::LINE_8);

/**
 * Convert 3D ellipsoid coordinates to Cartesian XYZ
 * */
inline cv::Vec3d ellipsoid_to_cart3d(double lat, double lon, double A, double B, double C)
{
  return cv::Vec3d(A * std::cos(lat) * std::cos(lon),
      B * std::cos(lat) * std::sin(lon),
      C * std::sin(lat));
}

/**
 * Convert 3D ellipsoid coordinates to Cartesian XY plane.
 * Return point visibility flag computed based on rotated surface normal direction.
 * */
inline bool ellipsoid_to_cart2d(double lat, double lon,
    double A, double B, double C,
    const cv::Matx33d & R,
    const cv::Point2d & center,
    cv::Point2d * pos)
{
  const cv::Vec3d v0 =
      ellipsoid_to_cart3d(lat, lon, A, B, C);

  const cv::Vec3d v1 =
      R * v0;

  pos->x = v1(0) + center.x;
  pos->y = v1(1) + center.y;

  /*
   * check point visibility based on rotated surface normal direction
   * */
  const double zr =
      R(2, 0) * v0(0) / (A * A) +
          R(2, 1) * v0(1) / (B * B) +
          R(2, 2) * v0(2) / (C * C);

  return  zr >= 0;
}


/**
 * Convert 3D ellipsoid coordinates to Cartesian XY plane.
 * Return point visibility flag computed based on rotated surface normal direction.
 * */
inline bool ellipsoid_to_cart2d(const cv::Vec3d & cart3d_pos,
    double A, double B, double C,
    const cv::Matx33d & R,
    const cv::Point2d & center,
    cv::Point2d * pos)
{
  const cv::Vec3d v1 =
      R * cart3d_pos;

  pos->x = v1(0) + center.x;
  pos->y = v1(1) + center.y;

  /*
   * check point visibility based on rotated surface normal direction
   * */
  const double zr =
      R(2, 0) * cart3d_pos(0) / (A * A) +
          R(2, 1) * cart3d_pos(1) / (B * B) +
          R(2, 2) * cart3d_pos(2) / (C * C);

  return  zr >= 0;
}


bool detect_saturn(cv::InputArray _image, cv::RotatedRect & output_bbox,
    cv::OutputArray output_mask = cv::noArray());

#endif /* __ellipsoid_h__ */
