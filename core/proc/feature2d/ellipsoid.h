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
 * Direct projection: from a 3D point on the planet to a 2D point on the screen.
 * Returns true if the point is on the visible (facing us) side of the planet.
 *  */
inline bool ellipsoid_to_cart2d(const cv::Vec3d & v,
    const cv::Point2d & center,
    double A, double B, double C,
    const cv::Matx33d & R,
    cv::Point2d & pos)
{
  const cv::Vec3d vcam = R * v;
  pos = cv::Point2d(vcam(0) + center.x, vcam(1) + center.y);
  // Visibility status
  return (vcam(2) <= 0.0);
}

/**
* Back projection from a 2D screen point to a 3D point on the planet's ellipsoid surface.
*
* @param pos Current 2D point on the screen (in pixels).
* @param center 2D center of the planet on the screen (in pixels).
* @param A Planet radius along the local X-axis (equatorial).
* @param B Planet radius along the local Y-axis (polar).
* @param C Planet radius along the local Z-axis (equatorial).
* @param R 3x3 rotation matrix (XYZscreen = R * XYZplanet).
* @param cart3d_pos Output vector of 3D coordinates on the planet (XYZplanet).
*   If the point is outside the disk, it is set to zero.
*/
inline bool ellipsoid_from_cart2d(const cv::Point2d & pos,
    const cv::Point2d & center,
    double A, double B, double C,
    const cv::Matx33d & R,
    cv::Vec3d & cart3d_pos )
{
  // XYZplanet = R.t() * XYZscreen.
  // Xp = R(0,0)*xs + R(1,0)*ys + R(2,0)*zs
  // Yp = R(0,1)*xs + R(1,1)*ys + R(2,1)*zs
  // Zp = R(0,2)*xs + R(1,2)*ys + R(2,2)*zs

  const double xs = pos.x - center.x;
  const double ys = pos.y - center.y;

  // Find zs from ellipsoid equation:
  //  (Xp/A)^2 + (Yp/B)^2 + (Zp/C)^2 = 1.
  const double x_stat = R(0, 0) * xs + R(1, 0) * ys;
  const double y_stat = R(0, 1) * xs + R(1, 1) * ys;
  const double z_stat = R(0, 2) * xs + R(1, 2) * ys;
  const double r_z_x = R(2, 0);
  const double r_z_y = R(2, 1);
  const double r_z_z = R(2, 2);
  const double invA2 = 1.0 / (A * A);
  const double invB2 = 1.0 / (B * B);
  const double invC2 = 1.0 / (C * C);

  // Quadratic equation
  // K2*zs^2 + 2*K1*zs + K0 = 0
  const double K2 = (r_z_x * r_z_x) * invA2 + (r_z_y * r_z_y) * invB2 + (r_z_z * r_z_z) * invC2;
  const double K1 = (x_stat * r_z_x) * invA2 + (y_stat * r_z_y) * invB2 + (z_stat * r_z_z) * invC2;
  const double K0 = (x_stat * x_stat) * invA2 + (y_stat * y_stat) * invB2 + (z_stat * z_stat) * invC2 - 1.0;
  const double discriminant = K1 * K1 - K2 * K0;
  if( discriminant < 0.0 ) {
    return false; // no hit the planet
  }

  // draw_ellipoid() considers points with pt_cam(2) <= 0 as visible,
  // Select the root that is closer to the observer (lower value along the Zscreen axis)
  const double sqrt_d = std::sqrt(discriminant);
  const double zs1 = (-K1 - sqrt_d) / K2;
  const double zs2 = (-K1 + sqrt_d) / K2;
  const double zs = std::min(zs1, zs2);

  // Convert a 3D point to screen coordinates and transform it into local planet coordinates
  cart3d_pos = R.t() * cv::Vec3d(xs, ys, zs);

  return true;
}



/**
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
 *   const cv::Matx33d R = build_ellipsoid_rotation(longitude_rotation, tilt_to_earth, position_angle);
 *   const double A = equatorial_radius1();
 *   const double B = equatorial_radius2();
 *   const double C = polar_radius();
 *   const cv::Point2f center = cv::Point2f(image_size.width / 2, image_size.height / 2);
 *   const cv::RotatedRect bbox = ellipsoid_bbox(center, A, B, C, R);
 *   cv::ellipse(image, bbox, cv::Scalar::all(255), 1, cv::LINE_AA);
 */
cv::RotatedRect ellipsoid_bbox(const cv::Point2f & center, double A, double B, double C, const cv::Matx33d & R);

inline cv::RotatedRect ellipsoid_bbox(const cv::Point2d & center, const cv::Vec3d & axes, const cv::Vec3d & pose)
{
  return ellipsoid_bbox(center, axes(0), axes(1), axes(2), build_ellipsoid_rotation(pose));
}

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
void draw_ellipsoid(cv::InputOutputArray image, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Matx33d & R,
    double lat_step, double lon_step,
    const cv::Scalar & color, int thickness, int line_type);

bool compute_ellipsoid_zrotation_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Matx33d & R1, const cv::Matx33d & R2,
    cv::Mat2f & rmap,
    cv::Mat1f & wmap,
    cv::Mat1b & rmask,
    double wscale = 1 );

cv::Rect ellipse_bounding_box(const cv::RotatedRect & rc);
cv::Rect ellipse_crop_box(const cv::RotatedRect & rc, const cv::Size & total_image_size, int margin = 1);


/**
 * Compute rotated 2D ellipse outline bound box,
 * appropriate to draw Saturn rings with cv::ellipse()
 * */
cv::RotatedRect rotated_ellipse_bbox(const cv::Point2f & center,
    double A, double B, const cv::Matx33d & R);

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
 * Create ellipse-shaped masks
 * */
void draw_ellipse_mask(cv::OutputArray image, const cv::Size & image_size, const cv::RotatedRect & rc);
void draw_ellipsoid_mask(cv::OutputArray image, const cv::Size & image_size,
    const cv::Point2d & center, const cv::Vec3d & axes, const cv::Vec3d & pose);

/*
 * Draw cv::RotatedRect
 * */
void draw_rotated_rect(cv::InputOutputArray _img, const cv::RotatedRect & rc,
    const cv::Scalar & color, int thickness, int line_type = cv::LINE_8);

std::string serialize_ellipsoid_to_string(const cv::Point2d & center,
    const cv::Vec3d & axes,  const cv::Vec3d & pose);

bool parse_ellipsoid_from_string(const std::string & s,
    cv::Point2d * center, cv::Vec3d * axes, cv::Vec3d * pose,
    bool * have_center = nullptr,
    bool * have_axes = nullptr,
    bool * have_pose = nullptr);

#endif /* __ellipsoid_h__ */
