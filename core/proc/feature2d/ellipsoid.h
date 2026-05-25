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
#include <core/proc/pose.h>



/**
 *
 * The coordinate system of the 2D image (frame) is taken as the initial (world) coordinate system:
 *  Axis X: directed from left to right (horizon of the frame).
 *  Axis Y: directed from top to bottom (vertical of the frame).
 *  Axis Z: directed from the observer into the screen (target / optical axis of the camera).
 *
 * Basic pose of the planet (before all rotations):
 *  The polar axis of the planet coincides with the Y axis of the frame,
 *  but is directed upwards (towards minus Y),
 *  and the equator lies in the XZ plane.
 *
 * Ry — The planet's proper rotation (Longitude).
 *  Since the polar axis is the Y axis, rotation around it spins the planet around its axis.
 *  Jupiter's texture will move along the equator (along the X axis).
 *
 * Rx — Sub-Earth latitude (Tilt to Earth).
 *  Rotation around the horizontal frame axis X.
 *  It tilts the north or south pole of the planet "towards us" (along the Z axis).
 *
 * Rz — Position angle (Tilt on the frame).
 *  Rotation around the optical axis Z.
 *  It simply spins the resulting disk of the planet in the plane of the screen, like a steering wheel.
 *
 * The chain of rotations is:
 *  R = Rz(position_angle) * Rx(tilt_to_earth) * Ry(longitude)
 *
*   Ry: Take the globe and rotate it around the polar axis, setting the desired central meridian (longitude).
*   Rx: Tilt the globe's polar axis forward or backward relative to the Earth,
*       simulating the sub-terrestrial latitude (orbital inclination).
*   Rz: Rotate the entire structure with the "steering wheel" in the plane of the sky to align
*       the axis with the true tilt of the north pole on the celestial sphere.
*
*  According to this semantics, the matrix R converts points from the local system of the planet
*  to the world system given by the screen :
*       XYZscreen = R * XYZplanet.
*  */

template<class _Tp>
inline cv::Vec<_Tp, 3> build_ellipsoid_pose(_Tp longitude_rotation, _Tp tilt_to_earth, _Tp position_angle)
{
  return cv::Vec<_Tp, 3> (longitude_rotation, tilt_to_earth, position_angle);
}

template<class _Tp>
inline cv::Matx<_Tp, 3, 3> build_ellipsoid_rotation(_Tp longitude_rotation, _Tp tilt_to_earth, _Tp position_angle)
{
  cv::Matx<_Tp, 3, 3> Rx, Ry, Rz;
  build_rotation(tilt_to_earth, longitude_rotation, position_angle, &Rx, &Ry, &Rz);
  return Rz * Rx * Ry;
}

template<class _Tp>
inline cv::Matx<_Tp, 3, 3> build_ellipsoid_rotation(const cv::Vec<_Tp, 3> & pose)
{
  return build_ellipsoid_rotation(
      pose(0),  // longitude_rotation
      pose(1),  // tilt_to_earth,
      pose(2)   // position_angle
      );
}

/**
 *
 * Given 3D ellipsoid with sem-axes A, B, C and pose specified by rotation matrix R
 * compute its outline (shadow) bounding box, appropriate for drawing
 * with cv::ellipse()
 *
 * https://math.stackexchange.com/questions/573055/projection-of-ellipsoid
 * https://www.r-5.org/files/books/computers/algo-list/image-processing/vision/Richard_Hartley_Andrew_Zisserman-Multiple_View_Geometry_in_Computer_Vision-EN.pdf
 * Hartley & Zisserman’s Multiple View Geometry In Computer Vision. Result 8.9 on page 201
 *
 * Example :
 *   const cv::Size image_size = image.size();
 *   const cv::Matx33d R = build_ellipsoid_pose(longitude_rotation, tilt_to_earth, position_angle);
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
 * Given 3D ellipsoid with sem-axes A, B, C and pose specified by rotation matrix R
 * draw it's coordinate grid and outline ellipse
 *
 * Center and axes specified in pixels.
 *
 * The pose matrix R must be as assigned by build_ellipsoid_rotation().
 *
 * The lat_step and lon_step are in radians.
 */
void draw_ellipoid(cv::InputOutputArray image, const cv::Point2f & center,
    const cv::Vec3d & axes, const cv::Matx33d & R,
    double lat_step, double lon_step,
    const cv::Scalar & color, int thickness, int line_type);


void draw_ellipoid(cv::InputOutputArray image, const cv::Point2f & center,
    const cv::Vec3d & axes, const cv::Vec3d & pose, double zrotation,
    double lat_step, double lon_step,
    const cv::Scalar & color, int thickness, int line_type);


bool compute_ellipsoid_zrotation_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Matx33d & R1, const cv::Matx33d & R2,
    cv::Mat2f & rmap,
    cv::Mat1f & wmap,
    cv::Mat1b & rmask,
    cv::Mat1f * rcounter = nullptr);
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
  const cv::Vec3d v0 = ellipsoid_to_cart3d(lat, lon, A, B, C);
  const cv::Vec3d v1 = R * v0;
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
  const cv::Vec3d v1 = R * cart3d_pos;
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


bool compute_ellipsoid_zrotation_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Vec3d & orientation, double zrotation,
    cv::Mat2f & output_rmap, cv::Mat1b & output_mask);

bool compute_saturn_zrotation_deltat_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Vec3d & orientation, double deltat_sec,
    cv::Mat2f & output_rmap, cv::Mat1b & output_mask);

#endif /* __ellipsoid_h__ */
