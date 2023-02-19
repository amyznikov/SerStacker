/*
 * estimate_image_transform.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#include "estimate_image_transform.h"
#include <core/debug.h>


namespace {

bool estimate_translation(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions)
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

    for( int iteration = 0; iteration < 10; ++iteration ) {

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

      for( uint i = 0; i < n; ++i ) {
        if( !blacklist[i] ) {

          const double dx =
              matched_current_positions[i].x - matched_reference_positions[i].x;

          const double dy =
              matched_current_positions[i].y - matched_reference_positions[i].y;

          if( fabs(dx - mx) > 3 * sx || fabs(dy - my) > 3 * sy ) {
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

  transform->set_translation(T);
  return true;
}

bool estimate_total_euclidean_transform(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions)
{

  if( matched_current_positions.size() < 2 ) {
    CF_ERROR("Not enough key point matches: %zu", matched_current_positions.size());
    return false;
  }

  const cv::Mat affine_matrix =
      cv::estimateAffinePartial2D(matched_reference_positions, matched_current_positions,
          cv::noArray(), cv::LMEDS, 7, 2000, 0.95, 10);

  if( affine_matrix.empty() ) {
    CF_ERROR("estimateAffinePartial2D() fails");
    return false;
  }

  // a:
  // [ ca * s   -sa * s  tx ]
  // [ sa * s    ca * s  ty ]

  const cv::Matx23f a =
      affine_matrix;

  const float scale =
      std::sqrt(a(0, 0) * a(0, 0) + a(0, 1) * a(0, 1));

  const float angle =
      std::atan2(a(1, 0), a(0, 0));

  const cv::Vec2f T(a(0, 2), a(1, 2));

  CF_DEBUG("Rotation: {\n"
         "%+g %+g \n"
         "%+g %+g \n"
         "}\n"
         "angle = %+g deg\n",
         a(0,0), a(0,1),
         a(1,0), a(1,1),
         angle * 180 / CV_PI);


     CF_DEBUG("Translation:{ %+g %+g}",
         T[0], T[1]);

  transform->set_translation(T);
  transform->set_rotation(angle);
  transform->set_scale(scale);
  return true;
}

// Finding optimal rotation and translation between corresponding 3D points
// <http://nghiaho.com/?page_id=671>
//
// @sa: https://en.wikipedia.org/wiki/Kabsch_algorithm
// @sa: https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
bool estimate_translation_and_rotation(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions)
{
  // matched_current_positions[i] =
  //  Rotation * matched_reference_positions[i] + Translation
  // a:
  // [ ca  -sa  tx ]
  // [ sa   ca  ty ]

  const std::vector<cv::Point2f> & cpoints1 = matched_reference_positions;
  const std::vector<cv::Point2f> & cpoints2 = matched_current_positions;
  cv::Mat1b mask;
  std::vector<float> residuals;

  cv::Matx22f Rotation;
  cv::Vec2f Translation;


  double cloud_scale = 0;
  double rmse = 0;
  int num_inliers = 0;

  static const auto toVec2f =
      [](const cv::Scalar & s) -> cv::Vec2f {
      return cv::Vec2f(s[0], s[1]);
  };

  mask.create(matched_current_positions.size(), 1);
  mask.setTo(255);

  for ( int iteration = 0; iteration < 10; ++iteration ) {

    const cv::Vec2f t1 =
        toVec2f(cv::mean(cv::Mat(cpoints1), mask));

    const cv::Vec2f t2 =
        toVec2f(cv::mean(cv::Mat(cpoints2), mask));

    cv::Matx22f C = cv::Matx22f::zeros();
    cv::Matx22f u, v, R;
    cv::Matx21f w;

    // Compute cloud scale and covariance matrix for input points.
    num_inliers = 0;
    for ( int k = 0, K = cpoints1.size(); k < K; ++k ) {
      if ( mask[k][0] ) {

        const cv::Vec2f p1(cpoints1[k].x - t1[0], cpoints1[k].y - t1[1]);
        const cv::Vec2f p2(cpoints2[k].x - t2[0], cpoints2[k].y - t2[1]);

        cloud_scale += cv::norm(p1, cv::NORM_L2SQR);
        cloud_scale += cv::norm(p2, cv::NORM_L2SQR);
        ++num_inliers;

        for ( int i = 0; i < 2; ++i ) {
          for ( int j = 0; j < 2; ++j ) {
            C(i, j) += p2[i] * p1[j];
          }
        }
      }
    }

    if ( num_inliers < 3 ) {
      CF_ERROR("ERROR: Not enough number of inliers: %d.\n"
          "At leat 3 points required\n",
          num_inliers);
      return false;
    }

    cloud_scale /= (2 * num_inliers);

    // Use SVD to compute R and T
    cv::SVD::compute(C, w, u, v);
    R = u * v;

    if ( cv::determinant(R) < 0 ) {
      R -= u.col(1) * (v.row(1) * 2.0);
    }

    const cv::Matx33f M(
        R(0, 0), R(0, 1), 0,
        R(1, 0), R(1, 1), 0,
        0,       0,       1);

    const cv::Matx33f T1(
        1, 0, -t1[0],
        0, 1, -t1[1],
        0, 0,  1);

    const cv::Matx33f T2(
        1, 0, t2[0],
        0, 1, t2[1],
        0, 0, 1);

    const cv::Matx33f RT =
        T2 * M * T1;

    (cv::Mat(RT)(cv::Rect(0, 0, 2, 2))).
        convertTo(Rotation, CV_32F,
        1. / RT(2, 2));

    (cv::Mat(RT)(cv::Rect(2, 0, 1, 2))).
        convertTo(Translation, CV_32F,
        1. / RT(2, 2));

    const float angle =
        std::atan2(Rotation(1, 0), Rotation(0, 0));

    CF_DEBUG("Rotation: {\n"
        "%+g %+g \n"
        "%+g %+g \n"
        "}\n"
        "angle = %+g deg\n"
        "Translation:{ %+g %+g}\n",
        Rotation(0,0), Rotation(0,1),
        Rotation(1,0), Rotation(1,1),
        angle * 180 / CV_PI,
        Translation[0], Translation[1]);

    transform->set_rotation(angle);
    transform->set_translation(Translation);

    // Compute RMSE
    rmse = 0;
    residuals.resize(cpoints1.size());

    for( int k = 0, K = cpoints1.size(); k < K; ++k ) {
      if( mask[k][0] ) {

        const cv::Vec2f p1(Rotation * cv::Vec2f(cpoints1[k].x, cpoints1[k].y) + Translation);
        const cv::Vec2f p2(cpoints2[k].x, cpoints2[k].y);
        const double residual = cv::norm(p2, p1, cv::NORM_L2SQR);
        rmse += residual;
        residuals[k] = residual;
      }
    }

    rmse /= num_inliers;

    CF_DEBUG("[%d] num_inliers=%d cloud_scale=%g rmse=%g", iteration, num_inliers,
        sqrt(cloud_scale), sqrt(rmse));


    const double rmse_threshold = 1e-6;
    if( rmse / cloud_scale < rmse_threshold ) {
      CF_DEBUG("[%d] break on small rmss  %g on threshold %g", iteration,
          sqrt(rmse / cloud_scale), sqrt(rmse_threshold));
      break;
    }

    int num_outliers = 0;
    for( int k = 0, K = cpoints1.size(); k < K; ++k ) {
      if( mask[k][0] ) {
        if( residuals[k] > 8 * rmse ) {
          mask[k][0] = 0;
          ++num_outliers;
        }
      }
    }

    if( num_outliers < 1 ) {
      CF_DEBUG("[%d] break on num_outliers=%d", iteration, num_outliers);
      return true;
    }
  }

  return true;
}
bool estimate_euclidean_transform(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_)
{

  const bool estimate_translation_only =
      !transform->fix_translation() && transform->fix_rotation() && transform->fix_scale();

  if( estimate_translation_only ) {

    return estimate_translation(transform,
        matched_current_positions_,
        matched_reference_positions_);
  }




  const bool estimate_translation_and_rotation_only =
      !transform->fix_translation() && !transform->fix_rotation() && transform->fix_scale();

  if ( estimate_translation_and_rotation_only ) {

    return estimate_translation_and_rotation(transform,
        matched_current_positions_,
        matched_reference_positions_);
  }




  const bool estimate_translation_rotation_and_scale =
      !transform->fix_translation() && !transform->fix_rotation() && !transform->fix_scale();

  if( true || estimate_translation_rotation_and_scale ) {

    return estimate_total_euclidean_transform(transform,
        matched_current_positions_,
        matched_reference_positions_);
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
    const std::vector<cv::Point2f> & matched_reference_positions_ )
{

  if( matched_current_positions_.size() < 3 ) {
    CF_ERROR("Not enough key points matches: %zu", matched_current_positions_.size());
    return false;
  }

  const cv::Mat affine_matrix =
      cv::estimateAffine2D(matched_reference_positions_, matched_current_positions_,
          cv::noArray(), cv::LMEDS, 7, 2000, 0.95, 10);

  if( affine_matrix.empty() ) {
    CF_ERROR("estimateAffine2D() fails");
    return false;
  }

  transform->set_affine_matrix(cv::Matx23f(affine_matrix));

  return true;
}

bool estimate_homography_transform(c_homography_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_ )
{

  if( matched_current_positions_.size() < 3 ) {
    CF_ERROR("Not enough key points matches: %zu", matched_current_positions_.size());
    return false;
  }

  const cv::Mat homography =
      cv::findHomography(matched_reference_positions_, matched_current_positions_,
          cv::LMEDS, 5, cv::noArray(), 2000, 0.95);

  if( homography.empty() ) {
    CF_ERROR("findHomography() fails");
    return false;
  }

  transform->set_homography_matrix(cv::Matx33f(homography));

  return true;
}

bool estimate_quadratic_transform(c_quadratic_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_ )
{
  const int N =
      matched_current_positions_.size();

  if( N < 6 ) {
    CF_ERROR("Not enough key points matches: %zu. Mininum 6 required",
        matched_current_positions_.size());
    return false;
  }

  /*
   * [x' y'] = [x y 1 x*y x*x y*y ] [ a00 ] [ a10 ]
   *                                [ a01 ] [ a11 ]
   *                                [ a02 ] [ a12 ]
   *                                [ a03 ] [ a13 ]
   *                                [ a04 ] [ a14 ]
   *                                [ a05 ] [ a15 ]
   *
   *
   * S2 = S1 * X
   */

  std::vector<uint8_t> inliers(N, true);

  int n = N;

  while (42) {

    cv::Mat1f S1(n, 6);
    cv::Mat1f S2(n, 2);

    for( int i = 0, j = 0; i < N; ++i ) {
      if( inliers[i] ) {

        const float x =
            matched_reference_positions_[i].x;

        const float y =
            matched_reference_positions_[i].y;

        S1[j][0] = x;
        S1[j][1] = y;
        S1[j][2] = 1;
        S1[j][3] = x * y;
        S1[j][4] = x * x;
        S1[j][5] = y * y;

        S2[j][0] = matched_current_positions_[i].x;
        S2[j][1] = matched_current_positions_[i].y;

        ++j;
      }
    }

    try {

      cv::Mat1f X;

      if( !cv::solve(S1, S2, X, cv::DECOMP_NORMAL) ) {
        CF_ERROR("cv::solve() fails");
        return false;
      }

      transform->set_matrix(X);

      if ( n > 6 ) {

        const cv::Mat1f D =
            S1 * X - S2;

        int outliers = 0;

        double s = 0;

        for( int i = 0, j = 0; i < n; ++i ) {
          if( inliers[i] ) {
            s += D[j][0] * D[j][0] + D[j][1] * D[j][1];
            ++j;
          }
        }

        s = s / (2 * n);

        for( int i = 0, j = 0; i < n; ++i ) {
          if( inliers[i] ) {
            const float d = D[j][0] * D[j][0] + D[j][1] * D[j][1];
            if( d > 9 * s ) {
              inliers[i] = false;
              ++outliers;
            }
            ++j;
          }
        }

        CF_DEBUG("outliers : %d / %d / %d", outliers, n, N);

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

} // namespace


bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_)
{
  if ( !transform ) {
    CF_ERROR("No image transform specified");
    return false;
  }

  if( c_translation_image_transform *t = dynamic_cast<c_translation_image_transform*>(transform) ) {
    return estimate_translation(t, matched_current_positions_, matched_reference_positions_);
  }

  if( c_euclidean_image_transform *t = dynamic_cast<c_euclidean_image_transform*>(transform) ) {
    return estimate_euclidean_transform(t, matched_current_positions_, matched_reference_positions_);
  }

  if( c_affine_image_transform *t = dynamic_cast<c_affine_image_transform*>(transform) ) {
    return estimate_affine_transform(t, matched_current_positions_, matched_reference_positions_);
  }

  if( c_homography_image_transform *t = dynamic_cast<c_homography_image_transform*>(transform) ) {
    return estimate_homography_transform(t, matched_current_positions_, matched_reference_positions_);
  }

  if( c_quadratic_image_transform *t = dynamic_cast<c_quadratic_image_transform*>(transform) ) {
    return estimate_quadratic_transform(t, matched_current_positions_, matched_reference_positions_);
  }

  CF_ERROR("Unknown c_image_transform trype specified (%s)",
      typeid(*transform).name());

  return false;
}
