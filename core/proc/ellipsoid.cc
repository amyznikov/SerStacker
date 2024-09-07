/*
 * ellipsoid.cc
 *
 *  Created on: Jul 9, 2024
 *      Author: amyznikov
 */

#include <core/proc/ellipsoid.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/pose.h>
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


cv::RotatedRect rotated_ellipse_bbox(const cv::Point2f & center, double A, double B, const cv::Matx33d & R)
{
  const double AA =
      1 / (A * A);

  const double BB =
      1 / (B * B);

  const double CC =
      1e9;

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


//  CF_DEBUG("\n"
//      "Qi: {\n"
//      " %+20g  %+20g %+20g %+20g\n"
//      " %+20g  %+20g %+20g %+20g\n"
//      " %+20g  %+20g %+20g %+20g\n"
//      " %+20g  %+20g %+20g %+20g\n"
//      "}\n",
//      Qi(0, 0), Qi(0, 1), Qi(0, 2), Qi(0, 3),
//      Qi(1, 0), Qi(1, 1), Qi(1, 2), Qi(1, 3),
//      Qi(2, 0), Qi(2, 1), Qi(2, 2), Qi(2, 3),
//      Qi(3, 0), Qi(3, 1), Qi(3, 2), Qi(3, 3));


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



void ellipse_poly(const cv::Point2f & center, const cv::Size2f & axes, double angle, std::vector<cv::Point> & pts)
{
  const double a =
      0.5 * axes.width;

  const double b =
      0.5 * axes.height;

  const double step =
      2. / std::max(a, b);

  const int nsteps =
      2 * CV_PI / step;

  const double ca =
      std::cos(angle);

  const double sa =
      std::sin(angle);

  for ( int i = 0; i < nsteps; ++i ) {

    const double t =
        i * step;

    const double x =
        a * std::cos(t);

    const double y =
        b * std::sin(t);

    const int ix =
        cvRound(center.x + x * ca - y * sa);

    const int iy =
        cvRound(center.y + x * sa + y * ca);

    pts.emplace_back(ix, iy);
  }
}

/*
 * Replacement for cv::ellipse() with better angular precision
 * */
void draw_ellipse(cv::InputOutputArray _img, const cv::RotatedRect & rc, const cv::Scalar & color, int thickness, int line_type)
{
  std::vector<cv::Point> pts;

  ellipse_poly(rc.center, rc.size, rc.angle * CV_PI / 180, pts);

  if( thickness < 0 ) {
    fillConvexPoly(_img, pts.data(), pts.size(), color, line_type);
  }
  else {
    const cv::Point * ppts[] = { pts.data() };
    const int npts[] = { (int)pts.size() };
    cv::polylines(_img, ppts, npts, 1, true, color, thickness, line_type, 0);
  }
}

void draw_rotated_rect(cv::InputOutputArray _img, const cv::RotatedRect & rc, const cv::Scalar & color, int thickness, int line_type)
{
  cv::Point vertices[4];
  cv::Point2f vertices2f[4];

  rc.points(vertices2f);

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

//
//
///**
// * */
//static bool ellipsoid_from_cart2d(
//    const cv::Point2d & pos,
//    double A, double B, double C,
//    const cv::Matx33d & R,
//    const cv::Point2d & center,
//    cv::Vec3d *  cart3d_pos )
//{
//
//  // (x/a)^2 + (y/b)^2 + (z/c)^2 = 1;
//  // (z/c)^2 = 1 - (x/a)^2 + (y/b)^2;
//  // (z/c)^2 = 1 - (x/a)^2 + (y/b)^2;
//
//
//
//
//
//
//
//



////  const cv::Point2d cpos(pos.x - center.x,
////      pos.y - center.y);
//
////  cv::Vec3d cart3d_pos(
////      pos.x - center.x,
////      pos.y - center.y,
////  );
//
////  const cv::Vec3d v1 =
////      R * cart3d_pos;
////
////  pos->x = v1(0) + center.x;
////  pos->y = v1(1) + center.y;
////
////  /*
////   * check point visibility based on rotated surface normal direction
////   * */
////  const double zr =
////      R(2, 0) * cart3d_pos(0) / (A * A) +
////          R(2, 1) * cart3d_pos(1) / (B * B) +
////          R(2, 2) * cart3d_pos(2) / (C * C);
////
////  return  zr >= 0;
//}


bool compute_ellipsoid_zrotation_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Vec3d & orientation, double zrotation,
    cv::Mat2f & rmap,
    cv::Mat1b & mask)
{
  rmap.create(size);
  rmap.setTo(cv::Vec2f::all(0));

  cv::Mat1f wmap(size, 0.0f);

  cv::Mat1f counter(size, 0.0f);

  const double & A =
      axes(0);

  const double & B =
      axes(1);

  const double & C =
      axes(2);

  const double L =
      std::max(A, std::max(B, C));

  const double lon_step =
      0.5 / L;

  const double lat_step =
      0.25 / L;

  const cv::Matx33d R1 =
      build_rotation2(orientation);

  const cv::Matx33d R2 =
      build_rotation2(orientation(0), orientation(1), orientation(2) + zrotation);

  cv::Point2d pos1, pos2;

  for( double lat = -CV_PI / 2; lat <= CV_PI / 2; lat += lat_step ) {

    for( double lon = 0; lon < 2 * CV_PI; lon += lon_step ) {

      const cv::Vec3d cart3d_pos =
          ellipsoid_to_cart3d(lat, lon, A, B, C);

      if( !ellipsoid_to_cart2d(cart3d_pos, A, B, C, R2, center, &pos2) ) {
        continue;
      }

      const int ix2 =
          cvRound(pos2.x);

      const int iy2 =
          cvRound(pos2.y);

      if( ix2 < 0 || ix2 >= size.width || iy2 < 0 || iy2 >= size.height ) {
        continue;
      }

      if( !ellipsoid_to_cart2d(cart3d_pos, A, B, C, R1, center, &pos1) ) {
        continue;
      }

      if( pos1.x < 0 || pos1.x >= size.width || pos1.y < 0 || pos1.y >= size.height ) {
        continue;
      }

      rmap[iy2][ix2][0] += pos1.x;
      rmap[iy2][ix2][1] += pos1.y;
      ++counter[iy2][ix2];
    }

  }

  for ( int y = 0; y < size.height; ++y ) {
    for ( int x = 0; x < size.width; ++x ) {
      if ( !counter[y][x] ) {
        rmap[y][x][0] = -1;
        rmap[y][x][1] = -1;
      }
      else {
        rmap[y][x][0] /= counter[y][x];
        rmap[y][x][1] /= counter[y][x];
      }
    }
  }

  cv::compare(counter, 0, mask, cv::CMP_GT);

  return true;
}

bool compute_saturn_zrotation_deltat_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Vec3d & orientation, double deltat_sec,
    cv::Mat2f & output_rmap, cv::Mat1b & output_mask)
{
  // Saturn daily rotation period is 10h 33m 38s.

  static constexpr double rotation_period_sec =
      10. * 3660 + 33. * 60 + 38.;

  const double rotation_angle_deg =
      360 * deltat_sec / rotation_period_sec;

  return compute_ellipsoid_zrotation_remap(size, center, axes,
      orientation,
      rotation_angle_deg * CV_PI / 180,
      output_rmap,
      output_mask);
}

