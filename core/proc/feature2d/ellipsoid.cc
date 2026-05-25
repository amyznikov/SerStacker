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
    cv::Mat1f & wmap,
    cv::Mat1b & rmask,
    cv::Mat1f * rcounter)
{
  rmap = cv::Mat2f::zeros(size);
  wmap = cv::Mat1f::ones(size);
  cv::Mat1f counter(size, 0.0f);

  const double A = axes(0); // Equatorial radius X
  const double B = axes(1); // Polar radius Y (Rotation axis)
  const double C = axes(2); // Equatorial radius Z

  const double invA2 = 1.0 / (A * A);
  const double invB2 = 1.0 / (B * B);
  const double invC2 = 1.0 / (C * C);

  const double L = std::max({A, B, C});
  const double lat_step = 0.25 / L;
  const double base_lon_step = 0.5 / L;

  const double CV_PI_2 = CV_PI / 2.0;

  for( double lat = -CV_PI_2; lat <= CV_PI_2; lat += lat_step ) {

    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);

    // Trigonometry optimization
    const double y_loc = B * sin_lat;
    const double A_cos = A * cos_lat;
    const double C_cos = C * cos_lat;

    // Local normal component along the Y axis (unchanged in the inner loop)
    const double ny_loc = y_loc * invB2;
    // The contribution of the local normal Y to the final camera normal R2
    const cv::Vec3d R2_col1_ny = cv::Vec3d(R2(0, 1), R2(1, 1), R2(2, 1)) * ny_loc;

    // Adaptive longitude step for the current latitude.
    // If we're near the pole (cos_lat is close to 0), the step approaches infinity.
    // We limit it from above to prevent freezing.
    const double current_lon_step = cos_lat > 0.01 ? base_lon_step / cos_lat : CV_2PI;

    // The Y-coordinate contribution to matrices R1 and R2
    const cv::Vec3d R1_col1_y = cv::Vec3d(R1(0, 1), R1(1, 1), R1(2, 1)) * y_loc;
    const cv::Vec3d R2_col1_y = cv::Vec3d(R2(0, 1), R2(1, 1), R2(2, 1)) * y_loc;
    const cv::Vec3d R1_col1_ny = cv::Vec3d(R1(0, 1), R1(1, 1), R1(2, 1)) * ny_loc;

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

      // WEIGHT CALCULATION

      // Local normal on the ellipsoid
      const double nx_loc = x_loc * invA2;
      const double nz_loc = z_loc * invC2;

      // Normal in the target camera system R2
      const double nc2_x = R2(0, 0) * nx_loc + R2_col1_ny(0) + R2(0, 2) * nz_loc;
      const double nc2_y = R2(1, 0) * nx_loc + R2_col1_ny(1) + R2(1, 2) * nz_loc;
      const double nc2_z = R2(2, 0) * nx_loc + R2_col1_ny(2) + R2(2, 2) * nz_loc;
      const double n2_len = std::sqrt(nc2_x * nc2_x + nc2_y * nc2_y + nc2_z * nc2_z);

      // Normal in the source camera system R1
      const double nc1_x = R1(0, 0) * nx_loc + R1_col1_ny(0) + R1(0, 2) * nz_loc;
      const double nc1_y = R1(1, 0) * nx_loc + R1_col1_ny(1) + R1(1, 2) * nz_loc;
      const double nc1_z = R1(2, 0) * nx_loc + R1_col1_ny(2) + R1(2, 2) * nz_loc;
      const double n1_len = std::sqrt(nc1_x * nc1_x + nc1_y * nc1_y + nc1_z * nc1_z);

      double weight = 0.0;
      if( n2_len > 1e-6 && n1_len > 1e-6 ) {
        const double cos_theta_target = std::abs(nc2_z) / n2_len;
        const double cos_theta_source = std::abs(nc1_z) / n1_len;
        weight = (cos_theta_target * cos_theta_target) * (cos_theta_source * cos_theta_source);
      }

      // Accumulate remap
      rmap[iy2][ix2][0] += (float)(pos1_x);
      rmap[iy2][ix2][1] += (float)(pos1_y);
      wmap[iy2][ix2] += (float)(weight);
      ++counter[iy2][ix2];
    }
  }

  // Final pass: average the pixels where there were intersections,
  // and fill the empty areas with default values ​​to preserve the frame's background.
  for (int y = 0; y < size.height; ++y) {
    for (int x = 0; x < size.width; ++x) {
      float cnt = counter[y][x];
      if (cnt < 1) {
        rmap[y][x][0] = x;
        rmap[y][x][1] = y;
      }
      else {
        rmap[y][x][0] /= cnt;
        rmap[y][x][1] /= cnt;
        wmap[y][x] /= cnt;
      }
    }
  }

  // Planetary disk mask after remap
  cv::compare(counter, 0, rmask, cv::CMP_GT);

  if ( rcounter ) {
    * rcounter = std::move(counter);
  }

  return true;
}

void draw_ellipoid(cv::InputOutputArray image, const cv::Point2f & center,
    const cv::Vec3d & axes, const cv::Vec3d & pose, double zrotation,
    double lat_step, double lon_step,
    const cv::Scalar & color, int thickness, int line_type)
{
  const double A = axes(0);
  const double B = axes(1);
  const double C = axes(2);

  const cv::Matx33d R =
      build_rotation2(pose(0), pose(1), pose(2) + zrotation);

  const cv::RotatedRect sbox =
      ellipsoid_bbox(center,
      A, B, C,
      R.t());

  if( lat_step > 0 ) {

    const double lon_step = 8 / std::max(sbox.size.width, sbox.size.height);

    cv::Point2d cpos, ppos;

    double lat, lon;

    for( lat = 0; lat < CV_PI / 2; lat += lat_step ) {

      ellipsoid_to_cart2d(-lat, lon = 0, A, B, C, R, center, &ppos);

      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if( ellipsoid_to_cart2d(-lat, lon, A, B, C, R, center, &cpos) ) {
          cv::line(image, ppos, cpos, color, 1, cv::LINE_AA);
        }
      }

      if ( lat == 0 ) {
        continue;
      }

      ellipsoid_to_cart2d(lat, lon = 0, A, B, C, R, center, &ppos);

      for( lon = lon_step; lon < 2 * CV_PI; lon += lon_step, ppos = cpos ) {
        if ( ellipsoid_to_cart2d(lat, lon, A, B, C, R, center, &cpos) ) {
          cv::line(image, ppos, cpos, color , 1, cv::LINE_AA);
        }
      }

    }
  }

  if( lon_step > 0 ) {

    const double lat_step = 8 / std::max(sbox.size.width, sbox.size.height);

    cv::Point2d cpos, ppos;
    double lat, lon;

    for( double lon = 0; lon < 2 * CV_PI; lon += lon_step ) {

      ellipsoid_to_cart2d(lat = -CV_PI / 2, lon, A, B, C, R, center, &ppos);

      for( lat = -CV_PI / 2 + lat_step; lat <= CV_PI / 2; lat += lat_step, ppos = cpos ) {
        if( ellipsoid_to_cart2d(lat, lon, A, B, C, R, center, &cpos) ) {
          cv::line(image, ppos, cpos, color, 1, cv::LINE_AA);
        }
      }
    }

  }

  //cv::ellipse(image, bbox, outline_color_, 1, cv::LINE_AA);
  draw_ellipse(image, sbox, color, 1, cv::LINE_AA);

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



bool compute_ellipsoid_zrotation_remap(const cv::Size & size, const cv::Point2d & center,
    const cv::Vec3d & axes, const cv::Vec3d & orientation, double zrotation,
    cv::Mat2f & rmap,
    cv::Mat1b & mask)
{
  rmap.create(size);
  rmap.setTo(cv::Vec2f::all(0));

  cv::Mat1f wmap(size, 0.0f);

  cv::Mat1f counter(size, 0.0f);

  const double A = axes(0);
  const double B = axes(1);
  const double C = axes(2);
  const double L = std::max(A, std::max(B, C));
  const double lon_step = 0.5 / L;
  const double lat_step = 0.25 / L;
  const cv::Matx33d R1 = build_rotation2(orientation);
  const cv::Matx33d R2 = build_rotation2(orientation(0), orientation(1), orientation(2) + zrotation);

  cv::Point2d pos1, pos2;

  for( double lat = -CV_PI / 2; lat <= CV_PI / 2; lat += lat_step ) {

    for( double lon = 0; lon < 2 * CV_PI; lon += lon_step ) {

      const cv::Vec3d cart3d_pos =
          ellipsoid_to_cart3d(lat, lon, A, B, C);

      if( !ellipsoid_to_cart2d(cart3d_pos, A, B, C, R2, center, &pos2) ) {
        continue;
      }

      const int ix2 = cvRound(pos2.x);
      const int iy2 = cvRound(pos2.y);
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

  for( int y = 0; y < size.height; ++y ) {
    for( int x = 0; x < size.width; ++x ) {
      if( !counter[y][x] ) {
        // Keep the rest of picture unchanged (remap to itself)
        rmap[y][x][0] = x;
        rmap[y][x][1] = y;
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

