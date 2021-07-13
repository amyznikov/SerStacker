/*
 * c_feature_based_image_registration.cc
 *
 *  Created on: Sep 18, 2019
 *      Author: amyznikov
 */

#include "c_feature_based_registration.h"
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
  : options_(opts)
{
}

c_feature_based_registration::c_feature_based_registration(const c_frame_registration_base_options & base_opts, const c_feature_based_registration_options & opts)
  : base(base_opts), options_(opts)
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

c_feature_based_registration_options & c_feature_based_registration::options()
{
  return options_;
}

const c_feature_based_registration_options & c_feature_based_registration::options() const
{
  return options_;
}

const cv::Ptr<cv::Feature2D> & c_feature_based_registration::set_keypoints_detector(const cv::Ptr<cv::Feature2D> & detector)
{
  return this->keypoints_detector_ = detector;
}

const cv::Ptr<cv::Feature2D> & c_feature_based_registration::keypoints_detector() const
{
  return this->keypoints_detector_;
}

void c_feature_based_registration::set_feature_hessian_threshold(double v)
{
  options_.hessianThreshold = v;
}

double c_feature_based_registration::feature_hessian_threshold() const
{
  return options_.hessianThreshold;
}


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

cv::Ptr<cv::Feature2D> c_feature_based_registration::create_keypoints_detector() const
{
//  static const struct {
//    int nfeatures = 150;
//    float scaleFactor = 1.2f;
//    int nlevels = 8;
//    int edgeThreshold = 31;
//    int firstLevel = 1;
//    int WTA_K = 4;
//    cv::ORB::ScoreType scoreType = cv::ORB::FAST_SCORE;
//    int patchSize = 31;
//    int fastThreshold = 8;
//  } ORB;
//
//  return cv::ORB::create(ORB.nfeatures,
//        ORB.scaleFactor,
//        ORB.nlevels,
//        ORB.edgeThreshold,
//        ORB.firstLevel,
//        ORB.WTA_K,
//        ORB.scoreType,
//        ORB.patchSize,
//        ORB.fastThreshold);


//  static const struct {
//    double hessianThreshold = 200;
//    int nOctaves = 4;
//    int nOctaveLayers = 1;
//    bool extended = false;
//    bool upright = true;
//  } SURF;

  return cv::xfeatures2d::SURF::create(
      options_.hessianThreshold,
      options_.nOctaves,
      options_.nOctaveLayers,
      options_.extended,
      options_.upright);
}



// Create image appropriate for sift/surf/orb/etc feature detection
// TODO: Check the Christopher Tsai "Effects of 2-D Preprocessing on Feature Extraction"
//    <https://pdfs.semanticscholar.org/185e/62d607becc8d0ee2409a224a36c58b084ab3.pdf>
bool c_feature_based_registration::create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk) const
{
  cv::Mat tmp;
  cv::normalize(src, tmp, 0, 255, cv::NORM_MINMAX, -1, srcmsk);
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

  if ( reference_descriptors_.depth() == CV_32F ) {
    keypoints_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
        cv::makePtr<cv::flann::KDTreeIndexParams>(1),
        cv::makePtr<cv::flann::SearchParams>(cvflann::FLANN_CHECKS_UNLIMITED, 0, false)); // 32
  }
  else {
    keypoints_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(
        //        cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2),
        cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1),
        cv::makePtr<cv::flann::SearchParams>(cvflann::FLANN_CHECKS_UNLIMITED, 0, false));
  }

  keypoints_matcher_->add(reference_descriptors_);
  keypoints_matcher_->train();

  return true;
}

bool c_feature_based_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return base::create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}

bool c_feature_based_registration::estimate_feature_transform(cv::InputArray current_feature_image,
    cv::InputArray current_feature_mask,
    cv::Mat1f * current_transform)
{
  keypoints_detector_->detectAndCompute(current_feature_image, current_feature_mask,
      current_keypoints_, current_descriptors_);

  CF_DEBUG("current_keypoints_.size()=%zu",
      current_keypoints_.size());

  if ( current_keypoints_.size() < 3 ) {
    CF_ERROR("Too few current key points detected : %zu", current_keypoints_.size());
    return false;
  }

  static constexpr double lowe_ratio = 0.8;

  current_matches_.clear(), current_matches12_.clear();
  if ( lowe_ratio <= 0 ) {
    keypoints_matcher_->match(current_descriptors_, current_matches_);
  }
  else {

    // David Lowe ratio test nearest/second nearest < ratio
    static const auto lowe_ratio_test =
        [](const std::vector<std::vector<cv::DMatch>> & matches12,
            std::vector<cv::DMatch> * good_matches,
            double lowe_ratio = 0.8)
        {
          for ( const std::vector<cv::DMatch> & m : matches12 ) {
            if ( m.size() == 1 || (m.size() == 2 && m[0].distance < lowe_ratio * m[1].distance) )
            good_matches->emplace_back(m[0]);
          }
        };


    keypoints_matcher_->knnMatch(current_descriptors_, current_matches12_, 2);
    lowe_ratio_test(current_matches12_, &current_matches_, lowe_ratio);
  }

  CF_DEBUG("current_matches_.size()=%zu",
      current_matches_.size());

  if ( current_matches_.size() < 3 ) {
    CF_ERROR("Too few key points matches found : %zu", current_matches_.size());
    return false;
  }

  extract_matched_positions(current_keypoints_, reference_keypoints_, current_matches_,
      &matched_current_positions_, &matched_reference_positions_);

  switch ( motion_type() ) {
  case ECC_MOTION_TRANSLATION :
    case ECC_MOTION_EUCLIDEAN :
    *current_transform = cv::estimateAffinePartial2D(matched_reference_positions_, matched_current_positions_,
        cv::noArray(), cv::LMEDS, 7, 2000, 0.95, 10);
    if ( current_transform->empty() ) {
      CF_ERROR("estimateAffinePartial2D() fails");
      return false;
    }
    break;

  case ECC_MOTION_AFFINE :
    case ECC_MOTION_QUADRATIC :
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

