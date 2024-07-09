/*
 * ellipsoid_bbox.cc
 *
 *  Created on: Jul 9, 2024
 *      Author: amyznikov
 */

#include "ellipsoid_bbox.h"
#include <core/debug.h>

/**
 * Given 3D ellipsoid with sem-axes A, B, C and rotated by matrix R
 * compute its outline (shadow) bounding box, appropriate for drawing
 * with cv::ellipse()
 *
 * https://math.stackexchange.com/questions/573055/projection-of-ellipsoid
 * https://www.r-5.org/files/books/computers/algo-list/image-processing/vision/Richard_Hartley_Andrew_Zisserman-Multiple_View_Geometry_in_Computer_Vision-EN.pdf
 * Hartley & Zisserman’s Multiple View Geometry In Computer Vision. Result 8.9 on page 201
 *
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
    const cv::Matx33d & R)
{
  const double AA =
      1 / (A * A);

  const double BB =
      1 / (B * B);

  const double CC =
      1 / (C * C);


  const cv::Matx44d RR(
      R(0,0), R(0,1), R(0,2), 0,
      R(1,0), R(1,1), R(1,2), 0,
      R(2,0), R(2,1), R(2,2), 0,
      0,      0,      0,      1
      );

  const cv::Matx44d Q =
      RR.t() * cv::Matx44d(
          AA, 0, 0, 0,
          0, BB, 0, 0,
          0, 0, CC, 0,
          0, 0, 0, -1
          ) * RR;

  const cv::Matx44d Qi =
      Q.inv();

  const cv::Matx34d P(
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 0, 1
  );

  const cv::Matx33d Conic =
      (P * Qi * P.t()).inv();

  const cv::Matx22d q(
      Conic(0, 0), Conic(0, 1),
      Conic(1, 0), Conic(1, 1));


  cv::Vec2d eigenvalues;
  cv::Matx22d eigenvectors;

  cv::eigen(q, eigenvalues, eigenvectors);

  const double t0 =
      std::atan2(eigenvectors(1, 1),
          eigenvectors(1, 0));

  const double eigen_axis_x =
      2 / std::sqrt(eigenvalues(1));

  const double eigen_axis_y =
      2 / std::sqrt(eigenvalues(0));

//  CF_DEBUG("q: {\n"
//      " %+20g %+20g\n"
//      " %+20g %+20g\n"
//      "}\n"
//      "eigenvalues= {%+20g %+20g}\n"
//      "eigenvectors= {\n"
//      "  %+20g %+20g\n"
//      "  %+20g %+20g\n"
//      "}\n"
//      "t0 = %+20g\n"
//      "aces={%g %g}\n"
//      "\n",
//      q(0,0), q(0,1),
//      q(1,0), q(1,1),
//      eigenvalues(0), eigenvalues(1),
//      eigenvectors(0,0), eigenvectors(0, 1),
//      eigenvectors(1,0), eigenvectors(1, 1),
//      t0 * 180 / CV_PI,
//      eigen_axis_x, eigen_axis_y);

  return cv::RotatedRect(center, cv::Size2f(eigen_axis_x, eigen_axis_y), t0 * 180 / CV_PI);
}

