/*
 * c_feature_based_image_registration.cc
 *
 *  Created on: Sep 18, 2019
 *      Author: amyznikov
 */

#include "c_feature_based_registration.h"
#include <core/proc/normalize.h>
#include <core/debug.h>


static void extract_matched_positions(const std::vector<cv::KeyPoint> & current_keypoints, const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<cv::DMatch > & matches,
    std::vector<cv::Point2f> * current_positions,
    std::vector<cv::Point2f> * reference_positions )
{
  current_positions->clear(), current_positions->reserve(matches.size());
  reference_positions->clear(), reference_positions->reserve(matches.size());
  for ( const auto & m : matches ) {
    current_positions->emplace_back(current_keypoints[m.queryIdx].pt);
    reference_positions->emplace_back(reference_keypoints[m.trainIdx].pt);
  }
}

c_feature_based_registration::c_feature_based_registration()
{
}

c_feature_based_registration::c_feature_based_registration(const c_feature_based_registration_options & opts)
  : feature_options_(opts)
{
}

c_feature_based_registration::c_feature_based_registration(const c_frame_registration_base_options & base_opts, const c_feature_based_registration_options & opts)
  : base(base_opts), feature_options_(opts)
{
}


c_feature_based_registration::ptr c_feature_based_registration::create()
{
  return c_feature_based_registration::ptr(new c_feature_based_registration());
}

c_feature_based_registration::ptr c_feature_based_registration::create(const c_feature_based_registration_options & opts)
{
  return c_feature_based_registration::ptr(new c_feature_based_registration(opts));
}

c_feature_based_registration::ptr c_feature_based_registration::create(const c_frame_registration_base_options & base_opts, const c_feature_based_registration_options & opts)
{
  return c_feature_based_registration::ptr(new c_feature_based_registration(base_opts, opts));
}

c_feature_based_registration_options & c_feature_based_registration::feature_options()
{
  return feature_options_;
}

const c_feature_based_registration_options & c_feature_based_registration::feature_options() const
{
  return feature_options_;
}

const c_feature2d::ptr  & c_feature_based_registration::set_keypoints_detector(const c_feature2d::ptr & detector)
{
  return this->keypoints_detector_ = detector;
}

const c_feature2d::ptr & c_feature_based_registration::keypoints_detector() const
{
  return this->keypoints_detector_;
}
//
//void c_feature_based_registration::set_feature_hessian_threshold(double v)
//{
//  feature_options_. hessianThreshold = v;
//}
//
//double c_feature_based_registration::feature_hessian_threshold() const
//{
//  return feature_options_.hessianThreshold;
//}


const std::vector<cv::KeyPoint> & c_feature_based_registration::reference_keypoints() const
{
  return reference_keypoints_;
}

const cv::Mat & c_feature_based_registration::reference_descriptors() const
{
  return reference_descriptors_;
}

const std::vector<cv::KeyPoint> & c_feature_based_registration::current_keypoints() const
{
  return current_keypoints_;
}

const cv::Mat & c_feature_based_registration::current_descriptors() const
{
  return current_descriptors_;
}

c_feature2d::ptr c_feature_based_registration::create_keypoints_detector() const
{
  return create_sparse_feature_detector(feature_options_.sparse_feature_extractor.detector);
}



// Create image appropriate for sift/surf/orb/etc feature detection
// TODO: Check the Christopher Tsai "Effects of 2-D Preprocessing on Feature Extraction"
//    <https://pdfs.semanticscholar.org/185e/62d607becc8d0ee2409a224a36c58b084ab3.pdf>
bool c_feature_based_registration::create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk) const
{
  cv::Mat tmp;
  normalize_minmax(src, tmp, 0, 255, srcmsk);
  if ( !extract_channel(tmp, dst, srcmsk, dstmsk, registration_channel(), base_options_.feature_scale, CV_8U, 1) ) {
    CF_ERROR("extract_channel(feature_channel_=%d) fails", registration_channel());
    return false;
  }
  return true;
}


bool c_feature_based_registration::extract_reference_features(cv::InputArray reference_feature_image,
    cv::InputArray reference_feature_mask)
{
  if ( !keypoints_detector_ && !set_keypoints_detector(create_keypoints_detector()) ) {
    CF_FATAL("c_feature_based_image_registration:: setup keypoints detector fails");
    return false;
  }

  CF_DEBUG("keypoints_detector_=%p (%s)",  keypoints_detector_.get(), keypoints_detector_ ? typeid(*keypoints_detector_.get()).name() : "");

  CF_DEBUG("reference_feature_image: %dx%d %d channels %d depth",
      reference_feature_image.cols(), reference_feature_image.rows(),
      reference_feature_image.channels(),
      reference_feature_image.depth());

  CF_DEBUG("reference_feature_mask: %dx%d %d channels %d depth",
      reference_feature_mask.cols(), reference_feature_mask.rows(),
      reference_feature_mask.channels(),
      reference_feature_mask.depth());

  CF_DEBUG("reference_keypoints_.clear()");
  reference_keypoints_.clear();

  CF_DEBUG("reference_descriptors_.release()");
  reference_descriptors_.release();

  CF_DEBUG("keypoints_detector_->detectAndCompute()");
  keypoints_detector_->detectAndCompute(reference_feature_image, reference_feature_mask,
      reference_keypoints_,
      reference_descriptors_);

  CF_DEBUG("reference_keypoints_.size()=%zu",
      reference_keypoints_.size());

  if ( reference_keypoints_.size() < 3 ) {
    CF_ERROR("Too few reference key points detected : %zu",
        reference_keypoints_.size());
    return false;
  }

  keypoints_matcher_ =
      create_sparse_feature_matcher(
          feature_options_.sparse_feature_matcher);

  if ( !keypoints_matcher_ ) {
    CF_ERROR("create_sparse_feature_matcher() fails");
    return false;
  }
//  if ( reference_descriptors_.depth() == CV_32F ) {
//    keypoints_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
//        cv::makePtr<cv::flann::KDTreeIndexParams>(1),
//        cv::makePtr<cv::flann::SearchParams>(cvflann::FLANN_CHECKS_UNLIMITED, 0, false)); // 32
//  }
//  else {
//    keypoints_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
//        //        cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2),
//        cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1),
//        cv::makePtr<cv::flann::SearchParams>(cvflann::FLANN_CHECKS_UNLIMITED, 0, false));
//  }

  keypoints_matcher_->train(reference_descriptors_);

  return true;
}

bool c_feature_based_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return base::create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}

bool c_feature_based_registration::detect_and_match_keypoints(
    cv::InputArray current_feature_image,
    cv::InputArray current_feature_mask,

    std::vector<cv::Point2f> & output_matched_current_positions,
    std::vector<cv::Point2f> & output_matched_reference_positions,

    std::vector<cv::KeyPoint> * _current_keypoints,
    cv::Mat * _current_descriptors,
    std::vector<cv::DMatch> * _current_matches,
    std::vector<std::vector<cv::DMatch> > * _current_matches12) const
{

  std::vector<cv::KeyPoint> local_current_keypoints;
  cv::Mat local_current_descriptors;
  std::vector<cv::DMatch> local_current_matches;
  std::vector<std::vector<cv::DMatch> > local_current_matches12;

  std::vector<cv::KeyPoint> & current_keypoints =
      _current_keypoints ?
          *_current_keypoints :
          local_current_keypoints;

  cv::Mat & current_descriptors =
      _current_descriptors ?
          *_current_descriptors :
          local_current_descriptors;

  std::vector<cv::DMatch> & current_matches =
      _current_matches ?
          *_current_matches :
          local_current_matches;

  std::vector<std::vector<cv::DMatch> > & current_matches12 =
      _current_matches12 ?
          *_current_matches12 :
          local_current_matches12;

  keypoints_detector_->detectAndCompute(current_feature_image, current_feature_mask,
      current_keypoints, current_descriptors);

  CF_DEBUG("current_keypoints.size()=%zu",
      current_keypoints.size());

  if ( current_keypoints.size() < 3 ) {
    CF_ERROR("Too few current key points detected : %zu", current_keypoints.size());
    return false;
  }

//  static constexpr double lowe_ratio = 0.8;
//
//  current_matches.clear(), current_matches12.clear();
//  if ( lowe_ratio <= 0 ) {
//    keypoints_matcher_->match(current_descriptors, current_matches);
//  }
//  else {
//
//    // David Lowe ratio test nearest/second nearest < ratio
//    static const auto lowe_ratio_test =
//        [](const std::vector<std::vector<cv::DMatch>> & matches12,
//            std::vector<cv::DMatch> * good_matches,
//            double lowe_ratio = 0.8)
//        {
//          for ( const std::vector<cv::DMatch> & m : matches12 ) {
//            if ( m.size() == 1 || (m.size() == 2 && m[0].distance < lowe_ratio * m[1].distance) )
//            good_matches->emplace_back(m[0]);
//          }
//        };
//
//
//    keypoints_matcher_->knnMatch(current_descriptors, current_matches12, 2);
//    lowe_ratio_test(current_matches12, &current_matches, lowe_ratio);
//  }

  keypoints_matcher_->match(current_descriptors,
      current_matches);

  CF_DEBUG("current_matches.size()=%zu",
      current_matches.size());

  extract_matched_positions(current_keypoints, reference_keypoints_, current_matches,
      &output_matched_current_positions, &output_matched_reference_positions);

  if ( current_matches.size() < 1 ) {
    CF_ERROR("No key points matches found");
    return false;
  }

  return true;
}

bool c_feature_based_registration::estimate_feature_transform(cv::InputArray current_feature_image,
    cv::InputArray current_feature_mask,
    cv::Mat1f * current_transform)
{
//  keypoints_detector_->detectAndCompute(current_feature_image, current_feature_mask,
//      current_keypoints_, current_descriptors_);
//
//  CF_DEBUG("current_keypoints_.size()=%zu",
//      current_keypoints_.size());
//
//  if ( current_keypoints_.size() < 3 ) {
//    CF_ERROR("Too few current key points detected : %zu", current_keypoints_.size());
//    return false;
//  }
//
//  static constexpr double lowe_ratio = 0.8;
//
//  current_matches_.clear(), current_matches12_.clear();
//  if ( lowe_ratio <= 0 ) {
//    keypoints_matcher_->match(current_descriptors_, current_matches_);
//  }
//  else {
//
//    // David Lowe ratio test nearest/second nearest < ratio
//    static const auto lowe_ratio_test =
//        [](const std::vector<std::vector<cv::DMatch>> & matches12,
//            std::vector<cv::DMatch> * good_matches,
//            double lowe_ratio = 0.8)
//        {
//          for ( const std::vector<cv::DMatch> & m : matches12 ) {
//            if ( m.size() == 1 || (m.size() == 2 && m[0].distance < lowe_ratio * m[1].distance) )
//            good_matches->emplace_back(m[0]);
//          }
//        };
//
//
//    keypoints_matcher_->knnMatch(current_descriptors_, current_matches12_, 2);
//    lowe_ratio_test(current_matches12_, &current_matches_, lowe_ratio);
//  }
//
//  CF_DEBUG("current_matches_.size()=%zu",
//      current_matches_.size());
//
//  extract_matched_positions(current_keypoints_, reference_keypoints_, current_matches_,
//      &matched_current_positions_, &matched_reference_positions_);
//
//  if ( current_matches_.size() < 1 ) {
//    CF_ERROR("Not enough key points matches: %zu", current_matches_.size());
//    return false;
//  }

  /////////////////
  bool fOk =
      detect_and_match_keypoints(current_feature_image, current_feature_mask,
          matched_current_positions_, matched_reference_positions_,
          &current_keypoints_, &current_descriptors_,
          &current_matches_, &current_matches12_);

  if ( !fOk ) {
    CF_ERROR("detect_and_match_keypoints() fails");
    return false;
  }


  ///////////////

  switch ( motion_type() ) {
  case ECC_MOTION_TRANSLATION :
    // TODO: mnove this code into to estimateTranslation2D()
    if ( matched_reference_positions_.size() == 1 ) {
      *current_transform = createTranslationTransform(
          matched_current_positions_[0].x - matched_reference_positions_[0].x,
          matched_current_positions_[0].y - matched_reference_positions_[0].y);
    }
    else if ( matched_reference_positions_.size() == 2 ) {
      *current_transform = createTranslationTransform(
          0.5 * (matched_current_positions_[0].x - matched_reference_positions_[0].x + matched_current_positions_[1].x - matched_reference_positions_[1].x),
          0.5 * (matched_current_positions_[0].y - matched_reference_positions_[0].y + matched_current_positions_[1].y - matched_reference_positions_[1].y));
    }
    else {

      const uint n = matched_current_positions_.size();
      std::vector<bool> blacklist(n, false);

      double mx = 0, my = 0, sx = 0, sy = 0;
      int c = 0;

      for ( int iteration = 0; iteration < 10; ++iteration ) {

        mx = 0, my = 0, sx = 0, sy = 0;
        c = 0;

        for ( uint i = 0; i < n; ++i ) {
          if ( !blacklist[i] ) {

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

        if ( c < 1 ) {
          CF_ERROR("PROBLEM: ALL FEATURES ARE BLACKLISTED ON ITEARATION %d", iteration);
          return false;
        }

        mx /= c;
        my /= c;
        sx = sqrt(fabs(sx / c - mx * mx));
        sy = sqrt(fabs(sy / c - my * my));

        int blacklisted = 0;

        for ( uint i = 0; i < n; ++i ) {
          if ( !blacklist[i] ) {

            const double dx =
                matched_current_positions_[i].x - matched_reference_positions_[i].x;

            const double dy =
                matched_current_positions_[i].y - matched_reference_positions_[i].y;

            if ( fabs(dx - mx) > 3 * sx || fabs(dy - my) > 3 * sy ) {
              blacklist[i] = true;
              ++blacklisted;
            }
          }
        }

        CF_DEBUG("FEATURES ITEARATION %d: c=%d mx=%g my=%g sx=%g sy=%g blacklisted=%d",
            iteration, c, mx, my, sx, sy, blacklisted);

        if ( !blacklisted ) {
          break;
        }
      }

      *current_transform = createTranslationTransform(mx, my);
    }

    break;

    case ECC_MOTION_EUCLIDEAN :
      if ( current_matches_.size() < 2 ) {
        CF_ERROR("Not enough key points matches: %zu", current_matches_.size());
        return false;
      }

    *current_transform = cv::estimateAffinePartial2D(matched_reference_positions_, matched_current_positions_,
        cv::noArray(), cv::LMEDS, 7, 2000, 0.95, 10);

    if ( current_transform->empty() ) {
      CF_ERROR("estimateAffinePartial2D() fails");
      return false;
    }

    break;


  case ECC_MOTION_EUCLIDEAN_SCALED : // FIXME: update this code to estimate ECC_MOTION_EUCLIDEAN_SCALED CORRECTLY !!!!
  case ECC_MOTION_AFFINE :
  case ECC_MOTION_QUADRATIC :

    if ( current_matches_.size() < 3 ) {
      CF_ERROR("Not enough key points matches: %zu", current_matches_.size());
      return false;
    }

    *current_transform = cv::estimateAffine2D(matched_reference_positions_, matched_current_positions_,
        cv::noArray(), cv::LMEDS, 7, 2000, 0.95, 10);

    if ( current_transform->empty() ) {
      CF_ERROR("estimateAffine2D() fails");
      return false;
    }

    if ( motion_type() == ECC_MOTION_QUADRATIC ) {
      *current_transform = expandAffineTransform(*current_transform, motion_type());
    }
    break;

  case ECC_MOTION_HOMOGRAPHY :

    if ( current_matches_.size() < 3 ) {
      CF_ERROR("Not enough key points matches: %zu", current_matches_.size());
      return false;
    }

    *current_transform = cv::findHomography(matched_reference_positions_, matched_current_positions_,
        cv::LMEDS, 5, cv::noArray(), 2000, 0.95);

    if ( current_transform->empty() ) {
      CF_ERROR("findHomography() fails");
      return false;
    }

    break;

  default :
    CF_ERROR("Invalid motion tyoe %d specified", motion_type());
    return false;
  }

  if ( feature_scale() != 1. ) {

    scaleTransform(motion_type(),
        *current_transform,
        1. / feature_scale());

  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

