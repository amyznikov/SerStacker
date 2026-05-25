/*
 * ellipsoid.cc
 *
 *  Created on: Jul 9, 2024
 *      Author: amyznikov
 */

#include <core/proc/feature2d/ellipsoid.h>
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/debug.h>

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
 *   const double A = equatorial_radius1(); // Along screen X axis
 *   const double B = polar_radius();  // Along screen Y axis
 *   const double C = equatorial_radius2(); // Along screen Z axis
 *   const cv::Point2f center = cv::Point2f(image_size.width / 2, image_size.height / 2);
 *   const cv::RotatedRect bbox = ellipsoid_bbox(center, A, B, C, R));
 *   cv::ellipse(image, bbox, cv::Scalar::all(255), 1, cv::LINE_AA);
 */
cv::RotatedRect ellipsoid_bbox(const cv::Point2f & center,
    double A, double B, double C,
    const cv::Matx33d & R)
{
  const double AA = 1 / (A * A); // For Equatorial radius X
  const double BB = 1 / (B * B); // For Polar radius Y (Axis of rotation)
  const double CC = 1 / (C * C); // For Equatorial radius  Z

  // Dual matrix of the ellipsoid quadric
  // R converts from the planet's local coordinates to the camera's coordinate system
  const cv::Matx44d RR(
      R(0, 0), R(0, 1), R(0, 2), 0,
      R(1, 0), R(1, 1), R(1, 2), 0,
      R(2, 0), R(2, 1), R(2, 2), 0,
      0, 0, 0, 1
      );

  // For the direct transformation matrix: RR * Q_local * RR.t()
  const cv::Matx44d Q =
      RR * cv::Matx44d(
          AA, 0, 0, 0,
          0, BB, 0, 0,
          0, 0, CC, 0,
          0, 0, 0, -1
          ) * RR.t();

  const cv::Matx44d Qi = Q.inv();

  // Matrix of orthographic projection onto the XY plane (Z of the frame goes to infinity)
  const cv::Matx34d P(
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 0, 1
      );

  const cv::Matx33d Conic = (P * Qi * P.t()).inv();


  // Extract the upper 2x2 submatrix responsible for the ellipse shape
  const cv::Matx22d q(
      Conic(0, 0), Conic(0, 1),
      Conic(1, 0), Conic(1, 1));

  // eigenvalues(0) - maximum (corresponds to the minimum axis of the ellipse - the polar axis)
  // eigenvalues(1) - minimum (corresponds to the maximum axis of the ellipse - the equatorial axis)
  cv::Vec2d eigenvalues;
  cv::Matx22d eigenvectors;
  cv::eigen(q, eigenvalues, eigenvectors);

  // Major axis (Equator on screen)
  const double eigen_axis_x = 2 / std::sqrt(eigenvalues(1));
  // Minor axis (Pole on screen)
  const double eigen_axis_y = 2 / std::sqrt(eigenvalues(0));

  // Extract the major axis angle (line 1 in eigenvectors corresponds to eigenvalues(1))
  // In OpenCV, the Y axis points downwards, atan2(y, x) gives the correct direction
  const double t0 = std::atan2(eigenvectors(1, 1), eigenvectors(1, 0));
  return cv::RotatedRect(center, cv::Size2f(eigen_axis_x, eigen_axis_y), t0 * 180 / CV_PI);
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
void draw_ellipoid(cv::InputOutputArray image, const cv::Point2f & center,
    const cv::Vec3d & axes, const cv::Matx33d & R,
    double lat_step, double lon_step,
    const cv::Scalar & color, int thickness, int line_type)
{
  const double A = axes(0); // Equatorial radius X
  const double B = axes(1); // Polar radius Y
  const double C = axes(2); // Equatorial radius Z
  const double CV_PI_2 = CV_PI / 2.0;

  const double max_radius = std::max({A, B, C});
  const double sample_step = (max_radius > 1.0) ? (0.5 / max_radius) : 0.02;

  const auto project_point = [&](double x_loc, double y_loc, double z_loc) -> cv::Point2f {
      const cv::Vec3d pt_cam = R * cv::Vec3d(x_loc, y_loc, z_loc);
      return cv::Point2f(static_cast<float>(pt_cam(0) + center.x),
          static_cast<float>(pt_cam(1) + center.y)
      );
    };


  std::vector<cv::Point> curve_pts;

  if ( lat_step > 0 ) {

    const auto draw_parallel =
        [&](double lat) {

          curve_pts.clear();

          const double sin_lat = std::sin(lat);
          const double cos_lat = std::cos(lat);
          const double y_loc = B * sin_lat;

          for( double lon = 0; lon <= CV_2PI + sample_step; lon += sample_step ) {
            const double x_loc = A * cos_lat * std::sin(lon);
            const double z_loc = C * cos_lat * std::cos(lon);
            const cv::Vec3d pt_cam = R * cv::Vec3d(x_loc, y_loc, z_loc);

            if( pt_cam(2) <= 0 ) {
              curve_pts.emplace_back(cvRound(pt_cam(0) + center.x), cvRound(pt_cam(1) + center.y));
            }
            else { // If the line goes to the opposite side of the planet, then interrupt it and draw the accumulated piece
              if( curve_pts.size() > 1 ) {
                cv::polylines(image, curve_pts, false, color, thickness, line_type);
              }
              curve_pts.clear();
            }
          }
          if( curve_pts.size() > 1 ) {
            cv::polylines(image, curve_pts, false, color, thickness, line_type);
          }
      };

    draw_parallel(0);
    for( double lat = lat_step; lat < CV_PI_2; lat += lat_step ) {
      draw_parallel(-lat);
      draw_parallel(+lat);
    }
  }

  if ( lon_step > 0 ) {

    curve_pts.clear();

    for( double lon = 0; lon < CV_2PI; lon += lon_step ) {

      const double sin_lon = std::sin(lon);
      const double cos_lon = std::cos(lon);

      for( double lat = -CV_PI_2; lat <= CV_PI_2 + sample_step; lat += sample_step ) {
        double x_loc = A * std::cos(lat) * sin_lon;
        double y_loc = B * std::sin(lat);
        double z_loc = C * std::cos(lat) * cos_lon;

        cv::Vec3d pt_cam = R * cv::Vec3d(x_loc, y_loc, z_loc);
        if( pt_cam(2) <= 0 ) {
          curve_pts.emplace_back(cvRound(pt_cam(0) + center.x), cvRound(pt_cam(1) + center.y));
        }
        else {
          // If the line goes to the opposite side of the planet, then interrupt it and draw the accumulated piece
          if( curve_pts.size() > 1 ) {
            cv::polylines(image, curve_pts, false, color, thickness, line_type);
          }
          curve_pts.clear();
        }
      }
      if( curve_pts.size() > 1 ) {
        cv::polylines(image, curve_pts, false, color, thickness, line_type);
      }
    }
  }

  draw_ellipse(image, ellipsoid_bbox(center, A, B, C, R), color, thickness, cv::LINE_AA);
}

/**
 * Given 2D image of 3D ellipsoid with sem-axes A, B, C and pose specified by rotation matrix R1.
 *
 * Compute image remap matrix of type cv::Mat1f to be used with cv::remap() to
 * apply generic geometrical transformation of the image corresponding to new ellipsoid pose R2.
 *
 * Designed to apply for daily planet derotation.
 *
 * The size is 2D frame size in pixels with the image of a planetary disk (jovian, mars or saturn).
 * The center is coordinate of the disk ellipse center in pixels.
 * The axes is sem-axes A, B, C.
 * The R1 is the pose of ellipse imaged on the frame.
 * The R2 is new pose (target) pose of ellipse image.
 *
 * The output rmap is the mapping to be applied to the image.
 * The output mask is planetary disk mask to allow limit the result of cv::remap() to
 * planetary disk only not touching the rest of the frame.
 *
 */
bool compute_ellipsoid_zrotation_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Matx33d & R1, const cv::Matx33d & R2,
    cv::Mat2f & rmap,
    //cv::Mat1f & wmap,
    cv::Mat1b & rmask,
    cv::Mat1f * rcounter)
{
  rmap = cv::Mat2f::zeros(size);
  cv::Mat1f counter(size, 0.0f);

  const double CV_PI_2 = CV_PI / 2.0;

  const double A = axes(0); // Equatorial radius X
  const double B = axes(1); // Polar radius Y (Rotation axis)
  const double C = axes(2); // Equatorial radius Z

  const double invB2 = 1.0 / (B * B);

  const double L = std::max({A, B, C});
  const double lat_step = 0.5 / L;
  const double lon_step = 0.5 / L;

  const auto process_lat =
      [&](double lat) {
        const double sin_lat = std::sin(lat);
        const double cos_lat = std::cos(lat);

        const double y_loc = B * sin_lat;
        const double A_cos = A * cos_lat;
        const double C_cos = C * cos_lat;
        const double current_lon_step = cos_lat > 0.01 ? lon_step / cos_lat : CV_2PI;

        // The Y-coordinate contribution to matrices R1 and R2
        const cv::Vec3d R1_col1_y = cv::Vec3d(R1(0, 1), R1(1, 1), R1(2, 1)) * y_loc;
        const cv::Vec3d R2_col1_y = cv::Vec3d(R2(0, 1), R2(1, 1), R2(2, 1)) * y_loc;

        for( double lon = 0; lon < CV_2PI; lon += current_lon_step ) {

          const double sin_lon = std::sin(lon);
          const double cos_lon = std::cos(lon);

          const double x_loc = A_cos * sin_lon;
          const double z_loc = C_cos * cos_lon;

          // Project to R2
          const double pt2_x = R2(0, 0) * x_loc + R2_col1_y(0) + R2(0, 2) * z_loc;
          const double pt2_y = R2(1, 0) * x_loc + R2_col1_y(1) + R2(1, 2) * z_loc;
          const double pt2_z = R2(2, 0) * x_loc + R2_col1_y(2) + R2(2, 2) * z_loc;
          if (pt2_z > 0) {
            continue; // Back (invisible) side in target pose
          }

          const int ix2 = cvRound(pt2_x + center.x);
          const int iy2 = cvRound(pt2_y + center.y);
          if (ix2 < 0 || ix2 >= size.width || iy2 < 0 || iy2 >= size.height) {
            continue;
          }

          // Project to R1
          const double pt1_x = R1(0, 0) * x_loc + R1_col1_y(0) + R1(0, 2) * z_loc;
          const double pt1_y = R1(1, 0) * x_loc + R1_col1_y(1) + R1(1, 2) * z_loc;
          const double pt1_z = R1(2, 0) * x_loc + R1_col1_y(2) + R1(2, 2) * z_loc;
          if (pt1_z > 0) {
            continue; // Back (invisible) side in initial  pose
          }

          const double pos1_x = pt1_x + center.x;
          const double pos1_y = pt1_y + center.y;
          if (pos1_x < 0 || pos1_x >= size.width || pos1_y < 0 || pos1_y >= size.height) {
            continue;
          }

          // Accumulate remap
          rmap[iy2][ix2][0] += (float)(pos1_x);
          rmap[iy2][ix2][1] += (float)(pos1_y);
          ++counter[iy2][ix2];
        }
      };

  process_lat(0);
  for( double lat = lat_step; lat <= CV_PI_2; lat += lat_step ) {
    process_lat(lat);
    process_lat(-lat);
  }

  // Final pass: average the pixels where there were intersections,
  // and fill the empty areas with default values ​​to preserve the frame's background.
  cv::parallel_for_(cv::Range(0, size.height),
      [&](const cv::Range & range) {
        for (int y = range.start; y < range.end; ++y) {
          const float * cnp = counter[y];
          cv::Vec2f * __restrict rmp = rmap[y];
          for (int x = 0; x < size.width; ++x) {
            const float c = cnp[x];
            if (c < 1) {
              rmp[x][0] = -1; //x;
          rmp[x][1] = -1;// y;
        }
        else {
          rmp[x][0] /= c;
          rmp[x][1] /= c;
        }
      }
    }
  });


  // Planetary disk mask after remap
  cv::compare(counter, 0, rmask, cv::CMP_GT);

  if ( rcounter ) {
    * rcounter = std::move(counter);
  }

  return true;
}

bool compute_ellipsoid_zrotation_wmap(const cv::Point2d & center,
    const cv::Vec3d & axes,  const cv::Vec3d & target_pose,
    const cv::Mat2f & rmap,
    cv::Mat1f & wmap)
{
  static const auto distanceTransformRowsOnly =
      [](cv::InputArray _src, cv::OutputArray _dst) {
        const int src_rows = _src.rows();
        const cv::Mat src = _src.getMat();
        cv::Mat dst = cv::Mat::zeros(_src.size(), CV_MAKETYPE(CV_32F, _src.channels()));
        cv::parallel_for_(cv::Range(0, src_rows),
            [&](const cv::Range & range) {
              for (int y = range.start; y < range.end; ++y) {
                cv::distanceTransform(src.row(y), dst.row(y),
                    cv::DIST_L2, cv::DIST_MASK_PRECISE,
                    CV_32F);
              }
            });

        _dst.move(dst);
      };

  cv::RotatedRect rrc;
  cv::Mat w = cv::Mat::zeros(rmap.size(), CV_8UC1);
  cv::Mat1b emask;

  rrc.center = center;
  rrc.size.width = 2 * axes(0);
  rrc.size.height = 2 * axes(1);
  rrc.angle = 0;

  cv::ellipse(w, rrc, cv::Scalar::all(255), -1, cv::LINE_AA);
  w.copyTo(emask);

  distanceTransformRowsOnly(w, w);
  cv::normalize(w, w, 0, 1, cv::NORM_MINMAX);

  const cv::Mat M = cv::getRotationMatrix2D(center, -target_pose(2) * 180 / CV_PI, 1);
  cv::warpAffine(w, w, M, w.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  cv::remap(w, w, rmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  cv::warpAffine(emask, emask, M, emask.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  cv::compare(emask, 250, emask, cv::CMP_GE);
  //w.setTo(cv::Scalar::all(1), ~emask);
  wmap = std::move(w);

  return true;
}
cv::RotatedRect rotated_ellipse_bbox(const cv::Point2f & center, double A, double B, const cv::Matx33d & R)
{
  const double AA = 1 / (A * A);
  const double BB = 1 / (B * B);
  const double CC = 1e9;

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

  const double t0 = std::atan2(eigenvectors(1, 1), eigenvectors(1, 0));
  const double eigen_axis_x = 2 / std::sqrt(eigenvalues(1));
  const double eigen_axis_y = 2 / std::sqrt(eigenvalues(0));

  return cv::RotatedRect(center, cv::Size2f(eigen_axis_x, eigen_axis_y), t0 * 180 / CV_PI);
}



void ellipse_poly(const cv::Point2f & center, const cv::Size2f & axes, double angle, std::vector<cv::Point> & pts)
{
  const double a = 0.5 * axes.width;
  const double b = 0.5 * axes.height;
  const double step = 2. / std::max(a, b);
  const int nsteps = 2 * CV_PI / step;
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);

  for ( int i = 0; i < nsteps; ++i ) {
    const double t = i * step;
    const double x = a * std::cos(t);
    const double y = b * std::sin(t);
    const int ix = cvRound(center.x + x * ca - y * sa);
    const int iy = cvRound(center.y + x * sa + y * ca);
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
