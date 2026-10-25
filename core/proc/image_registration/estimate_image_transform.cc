/*
 * estimate_image_transform.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#include "estimate_image_transform.h"
#include <core/settings/camera_settings.h>
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member * members_of<ROBUST_METHOD>()
{
  static const c_enum_member members[] = {
      { ROBUST_METHOD_LMEDS, "LMEDS", "least-median of squares algorithm" },
      { ROBUST_METHOD_RANSAC, "RANSAC", "RANSAC algorithm" },
      { ROBUST_METHOD_RHO, "RHO", "RHO algorithm" },
      { EMM_LMEDS },
  };

  return members;
}

bool estimate_translation(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  cv::Vec2f T;

  if( matched_reference_positions.size() == 1 ) {
    T[0] = matched_current_positions[0].x - matched_reference_positions[0].x;
    T[1] = matched_current_positions[0].y - matched_reference_positions[0].y;
  }

  else if( matched_reference_positions.size() == 2 ) {

    T[0] = 0.5 * (matched_current_positions[0].x - matched_reference_positions[0].x +
        matched_current_positions[1].x - matched_reference_positions[1].x);

    T[1] = 0.5 * (matched_current_positions[0].y - matched_reference_positions[0].y +
        matched_current_positions[1].y - matched_reference_positions[1].y);

  }
  else {

    const uint n = matched_current_positions.size();
    std::vector<bool> blacklist(n, false);

    double mx = 0, my = 0, sx = 0, sy = 0;
    int c = 0;

    for( int iteration = 0; iteration < opts.translation.max_iterations; ++iteration ) {

      mx = 0, my = 0, sx = 0, sy = 0;
      c = 0;

      for( uint i = 0; i < n; ++i ) {
        if( !blacklist[i] ) {

          const double dx =
              matched_current_positions[i].x - matched_reference_positions[i].x;

          const double dy =
              matched_current_positions[i].y - matched_reference_positions[i].y;

          mx += dx;
          my += dy;
          sx += dx * dx;
          sy += dy * dy;
          ++c;
        }
      }

      if( c < 1 ) {
        CF_ERROR("PROBLEM: ALL FEATURES ARE BLACKLISTED ON ITEARATION %d", iteration);
        return false;
      }

      mx /= c;
      my /= c;
      sx = sqrt(fabs(sx / c - mx * mx));
      sy = sqrt(fabs(sy / c - my * my));

      int blacklisted = 0;

      const double f = opts.translation.rmse_factor;

      for( uint i = 0; i < n; ++i ) {
        if( !blacklist[i] ) {

          const double dx =
              matched_current_positions[i].x - matched_reference_positions[i].x;

          const double dy =
              matched_current_positions[i].y - matched_reference_positions[i].y;

          if( fabs(dx - mx) > f * sx || fabs(dy - my) > f * sy ) {
            blacklist[i] = true;
            ++blacklisted;
          }
        }
      }

      CF_DEBUG("FEATURES ITEARATION %d: c=%d mx=%g my=%g sx=%g sy=%g blacklisted=%d",
          iteration, c, mx, my, sx, sy, blacklisted);

      if( !blacklisted ) {
        break;
      }
    }

    T[0] = mx;
    T[1] = my;
  }

  const cv::Vec2f & Cr = transform->destination_reference_point();
  const cv::Vec2f & Cc = transform->source_reference_point();
  transform->set_translation(T + Cr - Cc);
  return true;
}

bool estimate_total_euclidean_transform(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  if( matched_current_positions.size() < 2 ) {
    CF_ERROR("Not enough key point matches: %zu", matched_current_positions.size());
    return false;
  }

  const cv::Vec2f Cr = transform->destination_reference_point();
  const cv::Vec2f Cc = transform->source_reference_point();

  const cv::Mat affine_matrix =
      cv::estimateAffinePartial2D(matched_reference_positions, matched_current_positions,
          cv::noArray(),
          opts.scaled_euclidean.method,
          opts.scaled_euclidean.ransacReprojThreshold,
          opts.scaled_euclidean.maxIters,
          opts.scaled_euclidean.confidence,
          opts.scaled_euclidean.refineIters);

  if( affine_matrix.empty() ) {
    CF_ERROR("estimateAffinePartial2D() fails");
    return false;
  }

  // a:
  // [ ca * s   -sa * s  tx ]
  // [ sa * s    ca * s  ty ]

  const cv::Matx23f a = affine_matrix;
  const float scale = std::sqrt(a(0, 0) * a(0, 0) + a(0, 1) * a(0, 1));
  const float angle = std::atan2(a(1, 0), a(0, 0));
  const cv::Matx22f R(a(0, 0), a(0, 1), a(1, 0), a(1, 1));
  const cv::Vec2f T = cv::Vec2f(a(0, 2), a(1, 2)) + R * Cr - Cc;

  CF_DEBUG("\n"
      "angle = %+g deg scale=%g\n"
      "R: {\n"
        "%+g %+g \n"
        "%+g %+g \n"
      "}\n"
      "T: {%+g %+g}"
      "\n",
      angle * 180 / CV_PI, scale,
      R(0, 0), R(0, 1),
      R(1, 0), R(1, 1),
      T(0), T(1));


  transform->set_translation(T);
  transform->set_rotation(angle);
  transform->set_scale(scale);
  return true;
}

// Find optimal rotation and translation between corresponding 3D points
// <http://nghiaho.com/?page_id=671>
// @sa: https://en.wikipedia.org/wiki/Kabsch_algorithm
// @sa: https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
bool estimate_translation_and_rotation(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  const int npts = static_cast<int>(matched_current_positions.size());
  if (npts < 3) {
    CF_ERROR("ERROR: At least 3 points required. Given: %d\n", npts);
    return false;
  }

  const cv::Vec2f Cr = transform->destination_reference_point();
  const cv::Vec2f Cc = transform->source_reference_point();

  std::vector<uint8_t> inliers(npts, uint8_t(1));
  std::vector<float> residuals(npts, 0.0f);

  for ( int iteration = 0; iteration < opts.euclidean.max_iterations; ++iteration ) {

    cv::Vec2f sum_r(0.0f, 0.0f), sum_c(0.0f, 0.0f);
    int num_inliers = 0;

    for( int k = 0; k < npts; ++k ) {
      if( inliers[k] ) {
        sum_r += cv::Vec2f(matched_reference_positions[k].x, matched_reference_positions[k].y) - Cr;
        sum_c += cv::Vec2f(matched_current_positions[k].x, matched_current_positions[k].y) - Cc;
        ++num_inliers;
      }
    }

    if ( num_inliers < 3 ) {
      CF_ERROR("ERROR: Not enough number of inliers: %d.\n", num_inliers);
      return false;
    }

    cv::Matx22f C = cv::Matx22f::zeros();
    float cloud_scale = 0.0f;

    const cv::Vec2f t1 = sum_r / num_inliers;
    const cv::Vec2f t2 = sum_c / num_inliers;

    for ( int k = 0; k < npts; ++k ) {
      if ( inliers[k] ) {

        const cv::Vec2f p1 = cv::Vec2f(matched_reference_positions[k].x, matched_reference_positions[k].y) - Cr - t1;
        const cv::Vec2f p2 = cv::Vec2f(matched_current_positions[k].x, matched_current_positions[k].y) - Cc - t2;

        cloud_scale += (p1[0] * p1[0] + p1[1] * p1[1]);
        cloud_scale += (p2[0] * p2[0] + p2[1] * p2[1]);

        C(0, 0) += p2[0] * p1[0]; // 0
        C(0, 1) += p2[0] * p1[1]; // 1
        C(1, 0) += p2[1] * p1[0]; // 2
        C(1, 1) += p2[1] * p1[1]; // 3
      }
    }
    cloud_scale /= (2.0f * num_inliers);

    // Kabsch analytical algorithm without SVD
    const float det = C(0, 0) * C(1, 1) - C(0, 1) * C(1, 0);
    const float angle = det >= 0.0f ? std::atan2(C(1, 0) - C(0, 1), C(0, 0) * C(1, 1)) :
        std::atan2(-C(1, 0) - C(0, 1), -C(0, 0) * C(1, 1));

    const cv::Matx22f Rotation(std::cos(angle), -std::sin(angle),
        std::sin(angle), std::cos(angle));

    const cv::Vec2f Translation = (t2 + Cc) - Rotation * (t1 + Cr);
    transform->set_translation(Translation);
    transform->set_rotation(angle);

    // Compute RMSE to check for outliers
    float rmse = 0.0f;
    for ( int k = 0; k < npts; ++k ) {
      if ( inliers[k] ) {
        const cv::Vec2f p1(matched_reference_positions[k].x, matched_reference_positions[k].y);
        const cv::Vec2f p2(matched_current_positions[k].x, matched_current_positions[k].y);
        const cv::Vec2f prediction = Rotation * p1 + Translation;
        const float diff_x = p2[0] - prediction[0];
        const float diff_y = p2[1] - prediction[1];
        const float residual = diff_x * diff_x + diff_y * diff_y;
        rmse += residual;
        residuals[k] = residual;
      }
    }
    rmse /= num_inliers;

    const float rmse_threshold = opts.euclidean.rmse_threshold;
    if ( (rmse / cloud_scale) < rmse_threshold ) {
      break;
    }

    // Label outliers
    int num_outliers = 0;
    const float max_allowed_residual = 8.0f * rmse;
    for ( int k = 0; k < npts; ++k ) {
      if ( inliers[k] && residuals[k] > max_allowed_residual ) {
        inliers[k] = 0;
        ++num_outliers;
      }
    }

    if ( num_outliers < 1 ) {
      break;
    }
  }

  return true;
}

bool estimate_euclidean_transform(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{

  const bool estimate_translation_only =
      !transform->fix_translation() && transform->fix_rotation() && transform->fix_scale();

  if( estimate_translation_only ) {

    return estimate_translation(transform,
        matched_current_positions,
        matched_reference_positions,
        opts);
  }

  const bool estimate_translation_and_rotation_only =
      !transform->fix_translation() && !transform->fix_rotation() && transform->fix_scale();

  if ( estimate_translation_and_rotation_only ) {

    return estimate_translation_and_rotation(transform,
        matched_current_positions,
        matched_reference_positions,
        opts);
  }

  const bool estimate_translation_rotation_and_scale =
      !transform->fix_translation() && !transform->fix_rotation() && !transform->fix_scale();

  if( true || estimate_translation_rotation_and_scale ) {

    return estimate_total_euclidean_transform(transform,
        matched_current_positions,
        matched_reference_positions,
        opts);
  }


  CF_ERROR("APP BUG: Some constraints are not yet coded");

  const bool estimate_translation_and_scale =
      !transform->fix_translation() && transform->fix_rotation() && !transform->fix_scale();

  if (estimate_translation_and_scale) {
    return false;
  }

  return false;
}

bool estimate_affine_transform(c_affine_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  /**
   * Image affine transform:
   *  xx = x_ref - Crx
   *  yy = y_ref - Cry
   *  x_cur = a00 * xx + a01 * yy + a02 + Ccx
   *  y_cur = a10 * xx + a11 * yy + a12 + Ccy
   */

  if( matched_current_positions_.size() < 3 ) {
    CF_ERROR("Not enough key points matches: %zu", matched_current_positions_.size());
    return false;
  }

  const cv::Vec2f Cr = transform->destination_reference_point();
  const cv::Vec2f Cc = transform->source_reference_point();

  const cv::Mat affine_matrix =
      cv::estimateAffine2D(matched_reference_positions, matched_current_positions_,
          cv::noArray(), opts.affine.method,
          opts.affine.ransacReprojThreshold,
          opts.affine.maxIters,
          opts.affine.confidence,
          opts.affine.refineIters);

  if( affine_matrix.empty() ) {
    CF_ERROR("estimateAffine2D() fails");
    return false;
  }

  cv::Matx23f a = affine_matrix;
  const cv::Matx22f A(a(0, 0), a(0, 1), a(1, 0), a(1, 1));
  const cv::Vec2f T = cv::Vec2f(a(0, 2), a(1, 2)) + A * Cr - Cc;
  a(0, 2) = T(0);
  a(1, 2) = T(1);

  transform->set_matrix(a);

  return true;
}

bool estimate_homography_transform(c_homography_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  /**
   * Image homography transform relative to reference point [Crx, Cry]:
   *  xx = x_ref - Crx
   *  yy = y_ref - Cry
   *
   *  w  =  a20 * xx + a21 * yy + a22
   *  x_cur =  (a00 * xx + a01 * yy + a02) / w + Ccx
   *  y_cur =  (a10 * xx + a11 * yy + a12) / w + Ccy
   */

  if( matched_current_positions_.size() < 4 ) {
    CF_ERROR("Not enough key points matches: %zu", matched_current_positions_.size());
    return false;
  }

  const cv::Mat homography =
      cv::findHomography(matched_reference_positions, matched_current_positions_,
          opts.homography.method,
          opts.homography.ransacReprojThreshold,
          cv::noArray(),
          opts.homography.maxIters,
          opts.homography.confidence);

  if( homography.empty() ) {
    CF_ERROR("findHomography() fails");
    return false;
  }

  const cv::Vec2f Cr = transform->destination_reference_point();
  const cv::Vec2f Cc = transform->source_reference_point();

  const cv::Matx33f H =
      homography;

  const cv::Matx33f TCc(
      1.f, 0.f, -Cc[0],
      0.f, 1.f, -Cc[1],
      0.f, 0.f, 1.f
      );

  const cv::Matx33f TCr(
      1.f, 0.f, Cr[0],
      0.f, 1.f, Cr[1],
      0.f, 0.f, 1.f
      );

  const cv::Matx33f h = TCc * H * TCr;
  transform->set_matrix(h);

  return true;
}



bool estimate_semi_quadratic_transform(c_semi_quadratic_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  const int N = matched_current_positions.size();
  if( N < 4 ) {
    CF_ERROR("Not enough key points matches: %zu. Mininum 4 required",
        matched_current_positions.size());
    return false;
  }

  /*
   * Semi-quadratic image transform, 2x4 = 8 parameters are estimated
   * xx = x_ref - Crx
   * yy = y_ref - Cry
   * x_cur = a00 * xx + a01 * yy + a02 + a03 * xx * yy + Ccx
   * y_cur = a10 * xx + a11 * yy + a12 + a13 * xx * yy + Ccy
   *
   * [ x' y' ] = [x y 1 x*y ] [ a00 ] [ a10 ]
   *                          [ a01 ] [ a11 ]
   *                          [ a02 ] [ a12 ]
   *                          [ a03 ] [ a13 ]
   *
   *
   * S2 = S1 * X
   * CurPos = RefPos * X
   */

  std::vector<uint8_t> inliers(N, true);

  const cv::Vec2f Cr = transform->destination_reference_point();
  const cv::Vec2f Cc = transform->source_reference_point();

  int n = N;

  while (42) {

    cv::Mat1f S1(n, 4);
    cv::Mat1f S2(n, 2);

    const float Crx = Cr[0];
    const float Cry = Cr[1];
    const float Ccx = Cc[0];
    const float Ccy = Cc[1];

    for( int i = 0, j = 0; i < N; ++i ) {
      if( inliers[i] ) {

        const float xxr = matched_reference_positions[i].x - Crx;
        const float yyr = matched_reference_positions[i].y - Cry;
        const float xxc = matched_current_positions[i].x - Ccx;
        const float yyc = matched_current_positions[i].y - Ccy;

        S1[j][0] = xxr;
        S1[j][1] = yyr;
        S1[j][2] = 1.0f;
        S1[j][3] = xxr * yyr;
        S2[j][0] = xxc;
        S2[j][1] = yyc;

        ++j;
      }
    }

    try {

      cv::Mat1f X;

      if( !cv::solve(S1, S2, X, cv::DECOMP_NORMAL) ) {
        CF_ERROR("cv::solve() fails");
        return false;
      }

      transform->set_matrix(X.t());

      if ( n > 4 ) {
        const cv::Mat1f D = S1 * X - S2;
        int outliers = 0;
        double s = 0;
        for( int i = 0, j = 0; i < n; ++i ) {
          if( inliers[i] ) {
            s += D[j][0] * D[j][0] + D[j][1] * D[j][1];
            ++j;
          }
        }

        s = s / (2 * n);

        const double f = opts.semi_quadratic.rmse_factor * opts.semi_quadratic.rmse_factor;
        for( int i = 0, j = 0; i < n; ++i ) {
          if( inliers[i] ) {
            const float d = D[j][0] * D[j][0] + D[j][1] * D[j][1];
            if( d > f * s ) {
              inliers[i] = false;
              ++outliers;
            }
            ++j;
          }
        }

        CF_DEBUG("outliers : %d / %d / %d sigma=%g", outliers, n, N, sqrt(s));

        if( outliers && (n -= outliers) >= 6 ) {
          continue;
        }
      }

      return true;
    }
    catch( const cv::Exception &e ) {
      CF_ERROR("Exception in cv::solve() : %s", e.msg.c_str());
      return false;
    }
  }

  return false;
}


bool estimate_quadratic_transform(c_quadratic_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  const int N = matched_current_positions.size();
  if( N < 6 ) {
    CF_ERROR("Not enough key points matches: %zu. Mininum 6 required",
        matched_current_positions.size());
    return false;
  }

  /*
   * Image full quadratic transform, 12 parameters are estimated
   * xx = x_ref - Crx
   * yy = y_ref - Cry
   * x_cur =  a00 * xx + a01 * yy + a02 + a03 * xx * yy + a04 * xx * xx + a05 * yy * yy + Ccx
   * y_cur =  a10 * xx + a11 * yy + a12 + a13 * xx * yy + a14 * xx * xx + a15 * yy * yy + Ccy
   *
   * [ x' y' ] = [x y 1 x*y x*x y*y ] [ a00 ] [ a10 ]
   *                                  [ a01 ] [ a11 ]
   *                                  [ a02 ] [ a12 ]
   *                                  [ a03 ] [ a13 ]
   *                                  [ a04 ] [ a14 ]
   *                                  [ a05 ] [ a15 ]
   *
   *
   * S2 = S1 * X
   * CurPos = RefPos * X
   */

  std::vector<uint8_t> inliers(N, true);

  const cv::Vec2f Cr = transform->destination_reference_point();
  const cv::Vec2f Cc = transform->source_reference_point();

  int n = N;

  while (42) {

    cv::Mat1f S1(n, 6);
    cv::Mat1f S2(n, 2);

    const float Crx = Cr[0];
    const float Cry = Cr[1];
    const float Ccx = Cc[0];
    const float Ccy = Cc[1];

    for( int i = 0, j = 0; i < N; ++i ) {
      if( inliers[i] ) {

        const float xxr = matched_reference_positions[i].x - Crx;
        const float yyr = matched_reference_positions[i].y - Cry;
        const float xxc = matched_current_positions[i].x - Ccx;
        const float yyc = matched_current_positions[i].y - Ccy;
        S1[j][0] = xxr;
        S1[j][1] = yyr;
        S1[j][2] = 1;
        S1[j][3] = xxr * yyr;
        S1[j][4] = xxr * xxr;
        S1[j][5] = yyr * yyr;

        S2[j][0] = xxc;
        S2[j][1] = yyc;

        ++j;
      }
    }

    try {

      cv::Mat1f X;

      if( !cv::solve(S1, S2, X, cv::DECOMP_NORMAL) ) {
        CF_ERROR("cv::solve() fails");
        return false;
      }

      transform->set_matrix(X.t());

      if ( n > 6 ) {
        const cv::Mat1f D = S1 * X - S2;
        int outliers = 0;

        double s = 0;

        for( int i = 0, j = 0; i < n; ++i ) {
          if( inliers[i] ) {
            s += D[j][0] * D[j][0] + D[j][1] * D[j][1];
            ++j;
          }
        }

        s = s / (2 * n);

        const double f = opts.quadratic.rmse_factor * opts.quadratic.rmse_factor;

        for( int i = 0, j = 0; i < n; ++i ) {
          if( inliers[i] ) {
            const float d = D[j][0] * D[j][0] + D[j][1] * D[j][1];
            if( d > f * s ) {
              inliers[i] = false;
              ++outliers;
            }
            ++j;
          }
        }

        CF_DEBUG("outliers : %d / %d / %d sigma=%g", outliers, n, N, sqrt(s));

        if( outliers && (n -= outliers) >= 6 ) {
          continue;
        }
      }

      return true;
    }
    catch( const cv::Exception &e ) {
      CF_ERROR("Exception in cv::solve() : %s", e.msg.c_str());
      return false;
    }
  }

  return false;
}

// FIXME: estimate_epipolar_derotation Not updated for Cr/Cc
bool estimate_epipolar_derotation(c_epipolar_derotation_image_transform * transform,
    const std::vector<cv::Point2f> & _matched_current_positions,
    const std::vector<cv::Point2f> & _matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
//
//  if ( !opts ) {
//    CF_ERROR("No camera matrix specified, the estimation of epipolar derotation requires known camera matrix");
//    return false;
//  }
//

  cv::Vec3d A = opts.epipolar_derotation.initial_rotation * CV_PI / 180; //(0, 0, 0);
  cv::Vec3d T = opts.epipolar_derotation.initial_translation; // (0,0, 1);
  cv::Mat1b inliers;

  const cv::Matx33d & camera_matrix =
      opts.epipolar_derotation.camera_intrinsics.camera_matrix;

  bool fOk =
      lm_refine_camera_pose(A, T,
          camera_matrix,
          _matched_current_positions,
          _matched_reference_positions,
          inliers,
          &opts.epipolar_derotation.camera_pose);

  if ( !fOk ) {
    CF_ERROR("lm_refine_camera_pose2() fails");
    return false;
  }


  const cv::Matx33d homography =
      camera_matrix * build_rotation(-A) * camera_matrix.inv();

  transform->set_matrix(cv::Matx33f(homography));


  return true;
}

// FIXME: estimate_prh_transform Not updated for Cr/Cc
bool estimate_prh_transform(c_epipolar_derotation_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_,
    const c_estimate_image_transform_options * options)
{
  // c_prh_image_transform

  return false;
}


bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    const c_estimate_image_transform_options & opts)
{
  INSTRUMENT_REGION("");

  if ( !transform ) {
    CF_ERROR("No image transform specified");
    return false;
  }

  CF_DEBUG("matched_current_positions.size=%zu", matched_current_positions.size());

  try {

    if( c_translation_image_transform *t = dynamic_cast<c_translation_image_transform*>(transform) ) {
      return estimate_translation(t, matched_current_positions, matched_reference_positions, opts);
    }

    if( c_euclidean_image_transform *t = dynamic_cast<c_euclidean_image_transform*>(transform) ) {
      return estimate_euclidean_transform(t, matched_current_positions, matched_reference_positions, opts);
    }

    if( c_affine_image_transform *t = dynamic_cast<c_affine_image_transform*>(transform) ) {
      return estimate_affine_transform(t, matched_current_positions, matched_reference_positions, opts);
    }

    if( c_epipolar_derotation_image_transform *t = dynamic_cast<c_epipolar_derotation_image_transform*>(transform) ) {
      return estimate_epipolar_derotation(t, matched_current_positions, matched_reference_positions, opts);
    }

    if( c_homography_image_transform *t = dynamic_cast<c_homography_image_transform*>(transform) ) {
      return estimate_homography_transform(t, matched_current_positions, matched_reference_positions, opts);
    }

    if( c_semi_quadratic_image_transform *t = dynamic_cast<c_semi_quadratic_image_transform*>(transform) ) {
      return estimate_semi_quadratic_transform(t, matched_current_positions, matched_reference_positions, opts);
    }

    if( c_quadratic_image_transform *t = dynamic_cast<c_quadratic_image_transform*>(transform) ) {
      return estimate_quadratic_transform(t, matched_current_positions, matched_reference_positions, opts);
    }

    CF_ERROR("Unknown c_image_transform trype specified (%s)",
        typeid(*transform).name());
  }
  catch( const cv::Exception & e ) {

    CF_ERROR("OpenCV Exception caught in %s():\n"
        "%s\n"
        "%s() : %d\n"
        "file : %s\n",
        __func__,
        e.err.c_str(), ///< error description
        e.func.c_str(),///< function name. Available only when the compiler supports getting it
        e.line,///< line number in the source file where the error has occurred
        e.file.c_str()///< source file name where the error has occurred
        );
  }
  catch( const std::exception & e ) {
    CF_ERROR("std::exception caught in %s(): %s\n", __func__, e.what());
  }
  catch( ... ) {
    CF_ERROR("Unknown exception caught in %s()\n", __func__);
  }

  return false;
}

bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<cv::DMatch> & sparse_matches,
    const c_estimate_image_transform_options & opts)
{
  INSTRUMENT_REGION("");

  std::vector<cv::Point2f> current_positions, reference_positions;

  for( const cv::DMatch &m : sparse_matches ) {
    const cv::KeyPoint & ckp = current_keypoints[m.queryIdx];
    const cv::KeyPoint & rkp = reference_keypoints[m.trainIdx];
    current_positions.emplace_back(ckp.pt);
    reference_positions.emplace_back(rkp.pt);
  }

  bool transformEstimated =
      estimate_image_transform(transform,
          current_positions, reference_positions,
          opts);

  if ( transformEstimated ) {

    const size_t num_current_keypoints = current_keypoints.size();
    const size_t num_referece_keypoints = reference_keypoints.size();
    const size_t available_pts = std::min(num_current_keypoints, num_referece_keypoints);
    if( available_pts > sparse_matches.size() ) {

      std::vector<cv::Point2f> rpos, rrpos;

      rpos.reserve(reference_keypoints.size());
      for( const cv::KeyPoint & kp : reference_keypoints ) {
        rpos.emplace_back(kp.pt);
      }

      if( !transform->remap(rpos, rrpos) ) {
        CF_ERROR("transform->remap(rpos, rrpos) fails");
        return false;
      }

      const std::vector<cv::KeyPoint> & cpts = current_keypoints;
      const double eps = 1.5 * 1.5;

      current_positions.clear();
      reference_positions.clear();

      for( int i = 0, ni = (int) rrpos.size(); i < ni; ++i ) {
        const cv::Point2f rrp = rrpos[i];

        double d2min = DBL_MAX;
        int jmin = -1;

        for( int j = 0, nj = (int) cpts.size(); j < nj; ++j ) {
          const cv::Point2f cp = cpts[j].pt;
          const double d2 = (cp.x - rrp.x) * (cp.x - rrp.x) + (cp.y - rrp.y) * (cp.y - rrp.y);
          if( d2 < d2min ) {
            d2min = d2;
            jmin = j;
          }
        }

        if( jmin >= 0 && d2min < eps ) {
          current_positions.emplace_back(cpts[jmin].pt);
          reference_positions.emplace_back(rpos[i]);
        }
      }

      transformEstimated =
          estimate_image_transform(transform,
              current_positions, reference_positions,
              opts);
      if( !transformEstimated ) {
        CF_ERROR("Re-estimate image transform using %zu points fails", current_positions.size());
      }
      else {
        CF_DEBUG("Re-estimated using %zu point pairs", current_positions.size());
      }
    }
  }

  return transformEstimated;
}

bool serialize_image_transform_estimation_options(c_config_setting settings, bool save,
    c_estimate_image_transform_options & opts)
{
  if( auto group = SERIALIZE_GROUP(settings, save, "translation") ) {
    SERIALIZE_OPTION(group, save, opts.translation, rmse_factor);
    SERIALIZE_OPTION(group, save, opts.translation, max_iterations);
  }

  if( auto group = SERIALIZE_GROUP(settings, save, "euclidean") ) {
    SERIALIZE_OPTION(group, save, opts.euclidean, rmse_threshold);
    SERIALIZE_OPTION(group, save, opts.euclidean, max_iterations);
  }

  if( auto group = SERIALIZE_GROUP(settings, save, "scaled_euclidean") ) {
    SERIALIZE_OPTION(group, save, opts.scaled_euclidean, ransacReprojThreshold);
    SERIALIZE_OPTION(group, save, opts.scaled_euclidean, confidence);
    SERIALIZE_OPTION(group, save, opts.scaled_euclidean, method);
    SERIALIZE_OPTION(group, save, opts.scaled_euclidean, maxIters);
    SERIALIZE_OPTION(group, save, opts.scaled_euclidean, refineIters);
  }

  if( auto group = SERIALIZE_GROUP(settings, save, "affine") ) {
    SERIALIZE_OPTION(group, save, opts.affine, method);
    SERIALIZE_OPTION(group, save, opts.affine, ransacReprojThreshold);
    SERIALIZE_OPTION(group, save, opts.affine, maxIters);
    SERIALIZE_OPTION(group, save, opts.affine, confidence);
    SERIALIZE_OPTION(group, save, opts.affine, refineIters);
  }

  if( auto group = SERIALIZE_GROUP(settings, save, "homography") ) {
    SERIALIZE_OPTION(group, save, opts.homography, method);
    SERIALIZE_OPTION(group, save, opts.homography, ransacReprojThreshold);
    SERIALIZE_OPTION(group, save, opts.homography, maxIters);
    SERIALIZE_OPTION(group, save, opts.homography, confidence);
  }

  if( auto group = SERIALIZE_GROUP(settings, save, "semi_quadratic") ) {
    SERIALIZE_OPTION(group, save, opts.semi_quadratic, rmse_factor);
  }

  if( auto group = SERIALIZE_GROUP(settings, save, "quadratic") ) {
    SERIALIZE_OPTION(group, save, opts.quadratic, rmse_factor);
  }

  if( auto group = SERIALIZE_GROUP(settings, save, "epipolar_derotation") ) {
    //SERIALIZE_OPTION(group, save, opts.epipolar_derotation, camera_intrinsics);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation.camera_pose, robust_threshold);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation.camera_pose, epsf);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation.camera_pose, epsx);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation.camera_pose, max_iterations);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation.camera_pose, max_levmar_iterations);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation.camera_pose, direction);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation, initial_translation);
    SERIALIZE_OPTION(group, save, opts.epipolar_derotation, initial_rotation);
  }

  return true;
}


