/*
 * estimate_image_transform.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#include "estimate_image_transform.h"
#include <core/debug.h>


namespace {

bool estimate_euclidean_transform(c_euclidean_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_)
{
  const bool estimate_translation_only =
      !transform->fix_translation() && transform->fix_rotation() && transform->fix_scale();

  if( estimate_translation_only ) {

    cv::Vec2f T;

    if( matched_reference_positions_.size() == 1 ) {

      T[0] = matched_current_positions_[0].x - matched_reference_positions_[0].x;
      T[1] = matched_current_positions_[0].y - matched_reference_positions_[0].y;

    }

    else if( matched_reference_positions_.size() == 2 ) {

      T[0] = 0.5 * (matched_current_positions_[0].x - matched_reference_positions_[0].x +
          matched_current_positions_[1].x - matched_reference_positions_[1].x);

      T[1] = 0.5 * (matched_current_positions_[0].y - matched_reference_positions_[0].y +
          matched_current_positions_[1].y - matched_reference_positions_[1].y);

    }
    else {

      const uint n = matched_current_positions_.size();
      std::vector<bool> blacklist(n, false);

      double mx = 0, my = 0, sx = 0, sy = 0;
      int c = 0;

      for( int iteration = 0; iteration < 10; ++iteration ) {

        mx = 0, my = 0, sx = 0, sy = 0;
        c = 0;

        for( uint i = 0; i < n; ++i ) {
          if( !blacklist[i] ) {

            const double dx =
                matched_current_positions_[i].x - matched_reference_positions_[i].x;

            const double dy =
                matched_current_positions_[i].y - matched_reference_positions_[i].y;

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
                matched_current_positions_[i].x - matched_reference_positions_[i].x;

            const double dy =
                matched_current_positions_[i].y - matched_reference_positions_[i].y;

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


  const bool estimate_translation_rotation_and_scale =
      !transform->fix_translation() && !transform->fix_rotation() && !transform->fix_scale();

  if ( true || estimate_translation_rotation_and_scale ) {

    if( matched_current_positions_.size() < 2 ) {
      CF_ERROR("Not enough key points matches: %zu", matched_current_positions_.size());
      return false;
    }

    const cv::Mat affine_matrix =
        cv::estimateAffinePartial2D(matched_reference_positions_, matched_current_positions_,
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


    // convert to SerStacker representation of the translation
    /*
    s * ( ca * (x-ttx) - sa * ( y - tty) )
    s * ( sa * (x-ttx) + ca * ( y - tty) )


    s * ( ca * x - ca * ttx - sa * y + sa * tty )
    s * ( sa * x - sa * ttx + ca * y - ca * tty )

    s * ( ca * x - sa * y - ca * ttx + sa * tty )
    s * ( sa * x + ca * y - sa * ttx - ca * tty )

    s * ( ca * x - sa * y       - (ca * ttx - sa * tty) )
    s * ( sa * x + ca * y       - (sa * ttx + ca * tty) )


    tx = - s * ( ca * ttx - sa * tty )
    ty = - s * ( sa * ttx + ca * tty )

    tx = - ( s * ca * ttx - s * sa * tty )
    ty = - ( s * sa * ttx + s * ca * tty )

    -[tx] = ( s * ca  - s * sa ) [ ttx ]
    -[ty] = ( s * sa  + s * ca ) [ ttx ]

    [ttx] = - | s * ca  - s * sa |^-1  [tx]
    [tty] = - | s * sa  + s * ca |     [ty]

     */
    const float scale =
        std::sqrt(a(0, 0) * a(0, 0) + a(0, 1) * a(0, 1));

    const float angle =
        std::atan2(a(1, 0), a(1, 1));

    const cv::Vec2f T =
        cv::Matx22f(a(0, 0), a(0, 1), a(1, 0), a(1, 1)).inv() * cv::Vec2f(-a(0, 2), -a(1, 2));


    transform->set_translation(T);
    transform->set_rotation(angle);
    transform->set_scale(scale);
    return true;

  }


  CF_ERROR("APP BUG: Some constraints are not yet coded");


  const bool estimate_translation_and_rotation =
      !transform->fix_translation() && !transform->fix_rotation() && transform->fix_scale();

  if ( estimate_translation_and_rotation ) {



    return false;
  }


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

} // namespace


bool estimate_image_transform(c_image_transform * transform,
    const std::vector<cv::Point2f> & matched_current_positions_,
    const std::vector<cv::Point2f> & matched_reference_positions_)
{
  if ( !transform ) {
    CF_ERROR("No image transform specified");
    return false;
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
