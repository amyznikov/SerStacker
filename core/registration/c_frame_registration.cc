/*
 * c_frame_registration.cc
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#include "c_frame_registration.h"
#include <core/proc/normalize.h>
#include <core/get_time.h>
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


c_frame_registration::c_frame_registration() :
  ecch_(&ecc_)
{
}

c_frame_registration::c_frame_registration(const c_image_registration_options & options) :
    options_(options),
    ecch_(&ecc_)
{
}

const c_image_registration_status & c_frame_registration::status() const
{
  return current_status_;
}

c_image_registration_options & c_frame_registration::options()
{
  return options_;
}

const c_image_registration_options & c_frame_registration::options() const
{
  return options_;
}

const cv::Mat & c_frame_registration::reference_feature_image() const
{
  return reference_feature_image_;
}

const cv::Mat & c_frame_registration::reference_feature_mask() const
{
  return reference_feature_mask_;
}

const cv::Mat & c_frame_registration::reference_ecc_image() const
{
  return ecc_.reference_image();
}

const cv::Mat & c_frame_registration::reference_ecc_mask() const
{
  return ecc_.reference_mask();
}

const cv::Mat & c_frame_registration::current_feature_image() const
{
  return current_feature_image_;
}

const cv::Mat & c_frame_registration::current_feature_mask() const
{
  return current_feature_mask_;
}

const cv::Mat & c_frame_registration::current_ecc_image() const
{
  return ecc_.input_image();
}

const cv::Mat & c_frame_registration::current_ecc_mask() const
{
  return ecc_.input_mask();
}

const cv::Mat1f & c_frame_registration::current_transform() const
{
  return current_transform_;
}

const cv::Mat2f & c_frame_registration::current_remap() const
{
  return current_remap_;
}

c_sparse_feature_extractor::ptr c_frame_registration::create_keypoints_extractor() const
{
  return create_sparse_feature_extractor(options_.feature_registration.sparse_feature_extractor);
}

const c_sparse_feature_extractor::ptr & c_frame_registration::set_keypoints_extractor(const c_sparse_feature_extractor::ptr & extractor)
{
  return this->keypoints_extractor_ = extractor;
}

const c_sparse_feature_extractor::ptr & c_frame_registration::keypoints_extractor() const
{
  return this->keypoints_extractor_;
}

const c_ecc_forward_additive & c_frame_registration::ecc() const
{
  return this->ecc_;
}

const c_ecch & c_frame_registration::ecch() const
{
  return this->ecch_;
}

const c_ecch_flow & c_frame_registration::eccflow() const
{
  return this->eccflow_;
}

void c_frame_registration::set_enable_debug(bool v)
{
  enable_debug_ = v;
}

bool c_frame_registration::enable_debug() const
{
  return enable_debug_;
}

void c_frame_registration::set_debug_path(const std::string & v)
{
  debug_path_ = v;
}

const std::string& c_frame_registration::debug_path() const
{
  return debug_path_;
}

bool c_frame_registration::setup_referece_frame(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;

  reference_frame_size_ = reference_image.size();
  memset(&current_status_.timings, 0, sizeof(current_status_.timings));

  if( options_.feature_registration.enabled && options_.feature_registration.scale > 0 ) {

    if( !create_feature_image(reference_image, reference_mask, reference_feature_image_, reference_feature_mask_) ) {
      CF_ERROR("create_feature_image() fails");
      return false;
    }

    if( !extract_reference_features(reference_feature_image_, reference_feature_mask_) ) {
      CF_ERROR("extract_reference_features() fails");
      return false;
    }
  }

  if( options_.eccflow.enabled || (options_.ecc.enabled && options_.ecc.scale > 0) ) {

    if( !create_reference_ecc_image(reference_image, reference_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("create_reference_ecc_image() fails");
      return false;
    }

    if( options_.ecc.enabled && options_.ecc.scale > 0 ) {

      ecc_.set_eps(options_.ecc.eps);
      ecc_.set_min_rho(options_.ecc.min_rho);
      ecc_.set_input_smooth_sigma(options_.ecc.input_smooth_sigma);
      ecc_.set_reference_smooth_sigma(options_.ecc.reference_smooth_sigma);
      ecc_.set_update_step_scale(options_.ecc.update_step_scale);
      ecc_.set_max_iterations(options_.ecc.max_iterations);

      cv::Mat1f reference_ecc_image;
      cv::Mat1b reference_ecc_mask;

      if( options_.ecc.scale == 1 ) {
        reference_ecc_image = ecc_image;
        reference_ecc_mask = ecc_mask;
      }
      else if( options_.ecc.scale < 1 ) {
        cv::resize(ecc_image, reference_ecc_image, cv::Size(), options_.ecc.scale, options_.ecc.scale, cv::INTER_AREA);
      }
      else {
        cv::resize(ecc_image, reference_ecc_image, cv::Size(), options_.ecc.scale, options_.ecc.scale,
            cv::INTER_LINEAR_EXACT);
      }

      if( !ecc_mask.empty() && reference_ecc_mask.size() != reference_ecc_image.size() ) {
        cv::resize(ecc_mask, reference_ecc_mask, reference_ecc_image.size(), 0, 0, cv::INTER_AREA);
        cv::compare(reference_ecc_mask, 255, reference_ecc_mask, cv::CMP_GE);
      }

      if( !ecc_.set_reference_image(reference_ecc_image, reference_ecc_mask) ) {
        CF_ERROR("ecc_.set_reference_image() fails");
        return false;
      }
    }

    if( options_.eccflow.enabled ) {

      eccflow_.set_update_multiplier(options_.eccflow.update_multiplier);
      eccflow_.set_max_iterations(options_.eccflow.max_iterations);
      eccflow_.set_support_scale(options_.eccflow.support_scale);
      eccflow_.set_normalization_scale(options_.eccflow.normalization_scale);
      eccflow_.set_input_smooth_sigma(options_.eccflow.input_smooth_sigma);
      eccflow_.set_reference_smooth_sigma(options_.eccflow.reference_smooth_sigma);

      if( !eccflow_.set_reference_image(ecc_image, ecc_mask) ) {
        CF_ERROR("eccflow_.set_reference_image() fails");
        return false;
      }
    }
  }

  return true;
}

bool c_frame_registration::register_frame(cv::InputArray current_image, cv::InputArray current_mask,
    cv::OutputArray dst, cv::OutputArray dstmask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;

  double start_time = 0, total_time = 0;
  double t0, t1;

  bool have_transform = false;

  start_time = get_realtime_ms();

  current_frame_size_ = current_image.size();
  current_transform_ = createEyeTransform(options_.motion_type);
  memset(&current_status_.timings, 0, sizeof(current_status_.timings));

  /////////////////////////////////////////////////////////////////////////////
  if( options_.feature_registration.enabled && options_.feature_registration.scale > 0 ) {

    t0 = get_realtime_ms();
    if( !create_feature_image(current_image, current_mask, current_feature_image_, current_feature_mask_) ) {
      CF_ERROR("create_feature_image() fails");
      return false;
    }

    current_status_.timings.extract_feature_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if( !estimate_feature_transform(current_feature_image_, current_feature_mask_, &current_transform_) ) {
      CF_ERROR("estimate_feature_transform() fails");
      return false;
    }

    current_status_.timings.estimate_feature_transform =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }

  /////////////////////////////////////////////////////////////////////////////

  if( options_.eccflow.enabled || (options_.ecc.enabled && options_.ecc.scale > 0) ) {
    if( !create_current_ecc_image(current_image, current_mask, ecc_image, ecc_mask, 1) ) {
      CF_ERROR("extract_ecc_image(current_image) fails");
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  if( options_.ecc.enabled && options_.ecc.scale > 0 ) {

    t0 = get_realtime_ms();

    current_status_.timings.extract_ecc_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if( have_transform && options_.ecc.scale != 1 ) {

      scaleTransform(options_.motion_type,
          current_transform_,
          options_.ecc.scale);

    }

    cv::Mat1f current_ecc_image;
    cv::Mat1b current_ecc_mask;

    if( options_.ecc.scale == 1 ) {
      current_ecc_image = ecc_image;
      current_ecc_mask = ecc_mask;
    }
    else if( options_.ecc.scale < 1 ) {
      cv::resize(ecc_image, current_ecc_image, cv::Size(), options_.ecc.scale, options_.ecc.scale, cv::INTER_AREA);
    }
    else {
      cv::resize(ecc_image, current_ecc_image, cv::Size(), options_.ecc.scale, options_.ecc.scale,
          cv::INTER_LINEAR_EXACT);
    }

    if( !ecc_mask.empty() && current_ecc_mask.size() != current_ecc_image.size() ) {
      cv::resize(ecc_mask, current_ecc_mask, current_ecc_image.size(), 0, 0, cv::INTER_LINEAR);
      cv::compare(current_ecc_mask, 255, current_ecc_mask, cv::CMP_GE);
    }

    ecc_.set_motion_type(options_.motion_type);
    //ecch_.set_minimum_image_size(v)

    if( !ecc_.align_to_reference(current_ecc_image, current_transform_, current_ecc_mask) ) {
      CF_ERROR("ecc_.align_to_reference() fails: rho=%g/%g eps=%g/%g iterations=%d/%d",
          ecc_.rho(), ecc_.min_rho(),
          ecc_.current_eps(), ecc_.eps(),
          ecc_.num_iterations(), ecc_.max_iterations());
      return false;
    }

    if( options_.ecc.scale != 1 ) {

      scaleTransform(options_.motion_type,
          current_transform_,
          1. / options_.ecc.scale);
    }

    current_status_.timings.ecc_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    have_transform = true;
  }

  /////////////////////////////////////////////////////////////////////////////

  t0 = get_realtime_ms();

  createRemap(options_.motion_type,
      current_transform_,
      current_remap_,
      reference_frame_size_);

  current_status_.timings.create_remap =
      get_realtime_ms() - t0;

  if( options_.eccflow.enabled ) {

    t0 = get_realtime_ms();

    current_status_.timings.extract_smflow_image =
        (t1 = get_realtime_ms()) - t0, t0 = t1;

    if( !eccflow_.compute(ecc_image, current_remap_, ecc_mask) ) {
      CF_ERROR("smflow_.compute() fails");
      return false;
    }

    current_status_.timings.smflow_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;
  }

  if( dst.needed() || dstmask.needed() ) {

    t0 = get_realtime_ms();

    if( !remap(current_image, dst, current_mask, dstmask) ) {
      CF_ERROR("c_frame_registration::remap() fails");
      return false;
    }

    current_status_.timings.remap =
        (t1 = get_realtime_ms()) - t0;
  }

  total_time = get_realtime_ms() - start_time;

  CF_DEBUG("\nREGISTER: %g ms\n"
      "extract_feature_image: %g ms\n"
      "estimate_feature_transform: %g ms\n"
      "extract_ecc_image: %g ms\n"
      "ecc: rho=%g/%g eps=%g/%g iterations=%d/%d  %g ms (%g ms/it)\n"
      "create_remap: %g ms\n"
      "extract_smflow_image: %g ms\n"
      "smflow_align: %g ms\n"
      "remap : %g ms\n"
      "",
      total_time,
      current_status_.timings.extract_feature_image,
      current_status_.timings.estimate_feature_transform,
      current_status_.timings.extract_ecc_image,

      ecc_.rho(), ecc_.min_rho(),
      ecc_.current_eps(), ecc_.eps(),
      ecc_.num_iterations(), ecc_.max_iterations(),
      current_status_.timings.ecc_align, current_status_.timings.ecc_align / (ecc_.num_iterations() + 1),

      current_status_.timings.create_remap,
      current_status_.timings.extract_smflow_image,
      current_status_.timings.smflow_align,

      current_status_.timings.remap
      );

  return true;
}

// Create image appropriate for sift/surf/orb/etc feature detection
// TODO: Check the Christopher Tsai "Effects of 2-D Preprocessing on Feature Extraction"
//    <https://pdfs.semanticscholar.org/185e/62d607becc8d0ee2409a224a36c58b084ab3.pdf>
bool c_frame_registration::create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk) const
{
  cv::Mat tmp;

  if ( src.depth() == CV_8U ) {
    tmp = src.getMat();
  }
  else {
    normalize_minmax(src, tmp, 0, 255, srcmsk);
  }

  const bool fOk =
      extract_channel(tmp, dst, srcmsk, dstmsk,
          options_.registration_channel,
          options_.feature_registration.scale,
          CV_8U,
          1);

  if ( !fOk ) {
    CF_ERROR("extract_channel(channel_=%d scale=%g) fails",
        options_.registration_channel,
        options_.feature_registration.scale);
    return false;
  }

  return true;
}

bool c_frame_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  if( !extract_channel(src, dst, srcmsk, dstmsk, options_.registration_channel, scale, CV_32F) ) {
    CF_ERROR("extract_channel(registration_channel_=%d) fails", options_.registration_channel);
    return false;
  }

  if( options_.ecc.normalization_scale > 0 ) {

    cv::Mat1f &m = (cv::Mat1f&) dst.getMatRef();
    cv::Mat mean, stdev;
    cv::Mat mask;

    ecc_downscale(m, mean, options_.ecc.normalization_scale, cv::BORDER_REPLICATE);
    ecc_downscale(m.mul(m), stdev, options_.ecc.normalization_scale, cv::BORDER_REPLICATE);
    cv::absdiff(stdev, mean.mul(mean), stdev);
    cv::sqrt(stdev, stdev);

    ecc_upscale(mean, m.size());
    ecc_upscale(stdev, m.size());

    cv::add(stdev, options_.ecc.normalization_noise, stdev);
    cv::subtract(m, mean, m);
    cv::divide(m, stdev, m);
  }

  return true;
}

bool c_frame_registration::create_reference_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}

bool c_frame_registration::create_current_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}

bool c_frame_registration::extract_reference_features(cv::InputArray reference_feature_image, cv::InputArray reference_feature_mask)
{
  // generic
  if ( !keypoints_extractor_ && !set_keypoints_extractor(create_keypoints_extractor()) ) {
    CF_FATAL("c_feature_based_image_registration:: set_keypoints_extractor() fails");
    return false;
  }

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

  CF_DEBUG("keypoints_detector_->detectAndCompute(): keypoints_detector_=%p", keypoints_extractor_.get());
  keypoints_extractor_->detectAndCompute(reference_feature_image, reference_feature_mask,
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
      create_sparse_feature_matcher(options_.feature_registration.sparse_feature_matcher);

  if ( !keypoints_matcher_ ) {
    CF_ERROR("create_sparse_feature_matcher() fails");
    return false;
  }

  keypoints_matcher_->train(reference_descriptors_);


  // planetary disk
//  double gbsigma = 0.5;
//
//  if ( !options_.align_planetary_disk_masks ) {
//    if ( !simple_planetary_disk_detector(feature_image, feature_mask, &reference_centroid_, gbsigma) ) {
//      CF_FATAL("simple_small_planetary_disk_detector() fails");
//      return false;
//    }
//  }
//  else {
//    if ( !simple_planetary_disk_detector(feature_image, feature_mask, nullptr, gbsigma,
//        &reference_component_rect_, &reference_component_mask_, &reference_centroid_) ) {
//      CF_FATAL("simple_small_planetary_disk_detector() fails");
//      return false;
//    }
//
//    if ( enable_debug_ && !debug_path_.empty() ) {
//      save_image(reference_component_mask_,
//          ssprintf("%s/reference_component_mask_.tiff",
//              debug_path_.c_str()));
//    }
//
//  }
//
//  if ( feature_scale() != 1.0 && feature_scale() != 0 ) {
//    reference_centroid_ /= feature_scale();
//  }


  return true;
}

bool c_frame_registration::estimate_feature_transform(cv::InputArray current_feature_image, cv::InputArray current_feature_mask,
    cv::Mat1f * current_transform)
{
  // generic
  bool fOk =
      detect_and_match_keypoints(current_feature_image, current_feature_mask,
          matched_current_positions_, matched_reference_positions_,
          &current_keypoints_, &current_descriptors_,
          &current_matches_, &current_matches12_);

  if( !fOk ) {
    CF_ERROR("detect_and_match_keypoints() fails");
    return false;
  }

  ///////////////

  switch (options_.motion_type) {
  case ECC_MOTION_TRANSLATION:
    // TODO: mnove this code into to estimateTranslation2D()
    if( matched_reference_positions_.size() == 1 ) {
      *current_transform = createTranslationTransform(
          matched_current_positions_[0].x - matched_reference_positions_[0].x,
          matched_current_positions_[0].y - matched_reference_positions_[0].y);
    }
    else if( matched_reference_positions_.size() == 2 ) {
      *current_transform = createTranslationTransform(
          0.5
              * (matched_current_positions_[0].x - matched_reference_positions_[0].x + matched_current_positions_[1].x
                  - matched_reference_positions_[1].x),
          0.5
              * (matched_current_positions_[0].y - matched_reference_positions_[0].y + matched_current_positions_[1].y
                  - matched_reference_positions_[1].y));
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

      *current_transform = createTranslationTransform(mx, my);
    }

    break;

  case ECC_MOTION_EUCLIDEAN:
    if( current_matches_.size() < 2 ) {
      CF_ERROR("Not enough key points matches: %zu", current_matches_.size());
      return false;
    }

    *current_transform = cv::estimateAffinePartial2D(matched_reference_positions_, matched_current_positions_,
        cv::noArray(), cv::LMEDS, 7, 2000, 0.95, 10);

    if( current_transform->empty() ) {
      CF_ERROR("estimateAffinePartial2D() fails");
      return false;
    }

    break;

  case ECC_MOTION_EUCLIDEAN_SCALED: // FIXME: update this code to estimate ECC_MOTION_EUCLIDEAN_SCALED CORRECTLY !!!!
  case ECC_MOTION_AFFINE:
    case ECC_MOTION_QUADRATIC:

    if( current_matches_.size() < 3 ) {
      CF_ERROR("Not enough key points matches: %zu", current_matches_.size());
      return false;
    }

    *current_transform = cv::estimateAffine2D(matched_reference_positions_, matched_current_positions_,
        cv::noArray(), cv::LMEDS, 7, 2000, 0.95, 10);

    if( current_transform->empty() ) {
      CF_ERROR("estimateAffine2D() fails");
      return false;
    }

    if( options_.motion_type == ECC_MOTION_QUADRATIC ) {
      *current_transform =
          expandAffineTransform(*current_transform,
              options_.motion_type);
    }
    break;

  case ECC_MOTION_HOMOGRAPHY:

    if( current_matches_.size() < 3 ) {
      CF_ERROR("Not enough key points matches: %zu", current_matches_.size());
      return false;
    }

    *current_transform = cv::findHomography(matched_reference_positions_, matched_current_positions_,
        cv::LMEDS, 5, cv::noArray(), 2000, 0.95);

    if( current_transform->empty() ) {
      CF_ERROR("findHomography() fails");
      return false;
    }

    break;

  default:
    CF_ERROR("Invalid motion tyoe %d specified", options_.motion_type);
    return false;
  }

  if( options_.feature_registration.scale != 1 ) {

    scaleTransform(options_.motion_type,
        *current_transform,
        1. / options_.feature_registration.scale);

  }

  //  planetary disk
//
//  double gbsigma = 0.5;
//
//  if ( !options_.align_planetary_disk_masks ) {
//    if ( !simple_planetary_disk_detector(feature_image, feature_mask, &current_centroid_, gbsigma) ) {
//      CF_FATAL("simple_small_planetary_disk_detector() fails");
//      return false;
//    }
//  }
//  else {
//    if ( !simple_planetary_disk_detector(feature_image, feature_mask, nullptr, gbsigma,
//        &current_component_rect_, &current_component_mask_, &current_centroid_) ) {
//      CF_FATAL("simple_small_planetary_disk_detector() fails");
//      return false;
//    }
//
//    if ( enable_debug_ && !debug_path_.empty() ) {
//      save_image(current_component_mask_,
//          ssprintf("%s/current_component_mask_.tiff",
//              debug_path_.c_str()));
//    }
//  }
//
//  if ( feature_scale() != 1.0 && feature_scale() != 0 ) {
//    current_centroid_ /= feature_scale();
//  }
//
//  const cv::Point2f current_translation =
//      current_centroid_ - reference_centroid_;
//
//  *current_transform = createTranslationTransform(current_translation.x, current_translation.y);
//  if ( motion_type() != ECC_MOTION_TRANSLATION ) {
//    *current_transform = expandAffineTransform(current_transform_, motion_type());
//  }

  return true;
}

bool c_frame_registration::detect_and_match_keypoints(cv::InputArray current_feature_image,
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

  keypoints_extractor_->detectAndCompute(current_feature_image, current_feature_mask,
      current_keypoints, current_descriptors);

  CF_DEBUG("current_keypoints.size()=%zu",
      current_keypoints.size());

  if ( current_keypoints.size() < 3 ) {
    CF_ERROR("Too few current key points detected : %zu", current_keypoints.size());
    return false;
  }

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

bool c_frame_registration::custom_remap(const cv::Mat2f & rmap,
    cv::InputArray _src, cv::OutputArray dst,
    cv::InputArray _src_mask, cv::OutputArray dst_mask,
    enum ECC_INTERPOLATION_METHOD interpolation_flags,
    enum ECC_BORDER_MODE border_mode,
    const cv::Scalar & border_value) const
{
  cv::Mat src = _src.getMat();
  cv::Mat src_mask = _src_mask.getMat();

  cv::Size src_size;
  const cv::Scalar * border_value_ptr;

  if ( !src.empty() ) {
    src_size = src.size();
  }
  else if ( !src_mask.empty() ) {
    src_size = src_mask.size();
  }
  else {
    src_size = current_remap_.size();
  }


  if ( dst.needed() || dst_mask.needed()) {
    if ( current_remap_.size() != src_size ) {
      CF_ERROR("WARNING: src size (%dx%d) and remap size (%dx%d) different",
          src_size.width, src_size.height,
          current_remap_.cols, current_remap_.rows);
     // return false;
    }
  }

  if ( interpolation_flags < 0 ) {
    interpolation_flags = options_.interpolation;
  }

  if ( border_mode >= 0 ) {
    border_value_ptr = &border_value;
  }
  else {
    border_mode = options_.border_mode;
    border_value_ptr = &options_.border_value;
  }

  if ( dst.needed() ) {
    cv::remap(src, dst, rmap, cv::noArray(), interpolation_flags,
        border_mode, *border_value_ptr);
  }

  if ( dst_mask.needed() ) {

    if ( !src_mask.empty() ) {
      cv::remap(src_mask, dst_mask, rmap, cv::noArray(),
          interpolation_flags, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    }
    else {
      cv::remap(cv::Mat1b(src_size, 255), dst_mask, rmap, cv::noArray(),
          interpolation_flags, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    }

    cv::Mat & out_mask =
        dst_mask.getMatRef();

    if ( out_mask.depth() == CV_8U ) {
      // reduce mask edge artifacts
      cv::compare(out_mask, 255, out_mask, cv::CMP_GE);
      cv::erode(out_mask, out_mask, cv::Mat1b(5, 5, 255), cv::Point(-1, -1), 1,
          cv::BORDER_CONSTANT, cv::Scalar::all(255));
    }
  }

  return true;
}


bool c_frame_registration::remap(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray src_mask, cv::OutputArray dst_mask,
    enum ECC_INTERPOLATION_METHOD interpolation_flags,
    enum ECC_BORDER_MODE border_mode,
    const cv::Scalar & border_value) const
{
  return custom_remap(current_remap_,
      src, dst,
      src_mask, dst_mask,
      interpolation_flags,
      border_mode,
      border_value);
}

