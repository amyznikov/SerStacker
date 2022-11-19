/*
 * c_frame_registration.cc
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#include "c_frame_registration.h"
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/normalize.h>
#include <core/io/save_image.h>
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

const c_jovian_derotation & c_frame_registration::jovian_derotation() const
{
  return this->jovian_derotation_;
}

void c_frame_registration::set_ecc_image_preprocessor(const ecc_image_preprocessor_function & func)
{
  ecc_image_preprocessor_ = func;
}

const c_frame_registration::ecc_image_preprocessor_function & c_frame_registration::ecc_image_preprocessor() const
{
  return ecc_image_preprocessor_;
}

//void c_frame_registration::set_enable_debug(bool v)
//{
//  enable_debug_ = v;
//}
//
//bool c_frame_registration::enable_debug() const
//{
//  return enable_debug_;
//}

void c_frame_registration::set_debug_path(const std::string & v)
{
  debug_path_ = v;
}

const std::string& c_frame_registration::debug_path() const
{
  return debug_path_;
}

bool c_frame_registration::setup_reference_frame(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;
  cv::Mat eccflow_mask;

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

    //if( options_.jovian_derotation.enabled && options_.jovian_derotation.align_planetary_disk_masks ) {
    if( options_.ecc.replace_planetary_disk_with_mask ) {
      insert_planetary_disk_shape(ecc_image, ecc_mask, ecc_image, eccflow_mask);
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

      if ( !options_.ecc.enable_ecch ) {
        if( !ecc_.set_reference_image(reference_ecc_image, reference_ecc_mask) ) {
          CF_ERROR("ecc_.set_reference_image() fails");
          return false;
        }
      }
      else {
        if ( options_.ecc.ecch_minimum_image_size > 0 ) {
          ecch_.set_minimum_image_size(options_.ecc.ecch_minimum_image_size);
        }
        if( !ecch_.set_reference_image(reference_ecc_image, reference_ecc_mask) ) {
          CF_ERROR("ecch_.set_reference_image() fails");
          return false;
        }
      }
    }

    if( options_.eccflow.enabled ) {

      eccflow_.set_update_multiplier(options_.eccflow.update_multiplier);
      eccflow_.set_max_iterations(options_.eccflow.max_iterations);
      eccflow_.set_support_scale(options_.eccflow.support_scale);
      eccflow_.set_normalization_scale(options_.eccflow.normalization_scale);
      eccflow_.set_input_smooth_sigma(options_.eccflow.input_smooth_sigma);
      eccflow_.set_reference_smooth_sigma(options_.eccflow.reference_smooth_sigma);

      if (eccflow_mask.empty() ) {
        eccflow_mask = ecc_mask;
      }

      if( !eccflow_.set_reference_image(ecc_image, eccflow_mask) ) {
        CF_ERROR("eccflow_.set_reference_image() fails");
        return false;
      }
    }

  }


  if( options_.jovian_derotation.enabled ) {
    jovian_derotation_.set_min_rotation(options_.jovian_derotation.min_rotation);
    jovian_derotation_.set_max_rotation(options_.jovian_derotation.max_rotation);
    jovian_derotation_.set_normalization_scale(options_.jovian_derotation.ellipse.normalization_scale);
    jovian_derotation_.set_force_reference_ellipse(options_.jovian_derotation.ellipse.force_reference_ellipse);
    jovian_derotation_.set_normalization_blur(options_.jovian_derotation.ellipse.normalization_blur);
    jovian_derotation_.set_gradient_blur(options_.jovian_derotation.ellipse.gradient_blur);
    jovian_derotation_.set_eccflow_support_scale(options_.jovian_derotation.eccflow_support_scale);
    jovian_derotation_.set_eccflow_normalization_scale(options_.jovian_derotation.eccflow_normalization_scale);
    jovian_derotation_.set_eccflow_max_pyramid_level(options_.jovian_derotation.eccflow_max_pyramid_level);
    jovian_derotation_.set_hlines(options_.jovian_derotation.ellipse.hlines);

    jovian_derotation_.set_debug_path(debug_path_.empty() ? "" :
        ssprintf("%s/derotation-reference-frame", debug_path_.c_str()));

    if( !jovian_derotation_.setup_reference_image(reference_image, reference_mask) ) {
      CF_ERROR("jovian_derotation_.setup_reference_image() fails");
      return false;
    }
  }

  return true;
}

bool c_frame_registration::register_frame(cv::InputArray current_image, cv::InputArray current_mask,
    cv::OutputArray dst, cv::OutputArray dstmask)
{
  cv::Mat ecc_image;
  cv::Mat ecc_mask;
  cv::Mat eccflow_mask;

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

    //if( options_.jovian_derotation.enabled && options_.jovian_derotation.align_planetary_disk_masks ) {
    if( options_.ecc.replace_planetary_disk_with_mask ) {
      insert_planetary_disk_shape(ecc_image, ecc_mask, ecc_image, eccflow_mask);
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

    if ( !options_.ecc.enable_ecch ) {
      if( !ecc_.align_to_reference(current_ecc_image, current_transform_, current_ecc_mask) ) {
        CF_ERROR("ecc_.align_to_reference() fails: rho=%g/%g eps=%g/%g iterations=%d/%d",
            ecc_.rho(), ecc_.min_rho(),
            ecc_.current_eps(), ecc_.eps(),
            ecc_.num_iterations(), ecc_.max_iterations());
        return false;
      }
    }
    else {
      if( !ecch_.align_to_reference(current_ecc_image, current_transform_, current_ecc_mask) ) {
        CF_ERROR("ecch_.align_to_reference() fails: rho=%g/%g eps=%g/%g iterations=%d/%d",
            ecc_.rho(), ecc_.min_rho(),
            ecc_.current_eps(), ecc_.eps(),
            ecc_.num_iterations(), ecc_.max_iterations());
        return false;
      }
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

    if ( eccflow_mask.empty() ) {
      eccflow_mask = ecc_mask;
    }

    if( !eccflow_.compute(ecc_image, current_remap_, eccflow_mask) ) {
      CF_ERROR("smflow_.compute() fails");
      return false;
    }

    current_status_.timings.smflow_align =
        (t1 = get_realtime_ms()) - t0, t0 = t1;
  }

  if( options_.jovian_derotation.enabled ) {


    jovian_derotation_.set_debug_path(debug_path_.empty() ? "" :
        ssprintf("%s/derotation", debug_path_.c_str()));

    if ( !jovian_derotation_.compute(current_image, current_mask) ) {
      CF_ERROR("jovian_derotation_.compute() fails");
      return false;
    }

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

  if ( ecc_image_preprocessor_ ) {
    ecc_image_preprocessor_(dst.getMatRef(), dstmsk.getMatRef());
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

bool c_frame_registration::insert_planetary_disk_shape(const cv::Mat & src_ecc_image, const cv::Mat & src_mask,
    cv::Mat & dst_ecc_image, cv::Mat & dst_ecc_mask) const
{
  cv::Mat planetary_disk_mask;

  bool fOk =
      simple_planetary_disk_detector(src_ecc_image, src_mask,
          nullptr,
          2,
          options_.ecc.planetary_disk_mask_stdev_factor,
          nullptr,
          &planetary_disk_mask);

  if( !fOk ) {
    CF_ERROR("simple_small_planetary_disk_detector() fails");
    return false;
  }

//  if( options_.ecc.planetary_disk_mask_stdev_factor > 0 ) {
//
//    cv::Scalar m, s;
//
//    cv::meanStdDev(src_ecc_image, m, s, src_mask);
//
//    const double threshold =
//        s[0] * options_.ecc.planetary_disk_mask_stdev_factor;
//
//    cv::bitwise_and(planetary_disk_mask, src_ecc_image > threshold, planetary_disk_mask);
//  }
//  morphological_smooth_close(planetary_disk_mask, planetary_disk_mask, cv::Mat1b(3, 3, 255));
//  geo_fill_holes(planetary_disk_mask, planetary_disk_mask, 8);


  src_ecc_image.copyTo(dst_ecc_image);

  if ( !src_mask.empty() ) {
    cv::bitwise_and(src_mask, ~planetary_disk_mask, dst_ecc_mask );
  }
  else {
    cv::bitwise_not(planetary_disk_mask, dst_ecc_mask);
  }

  if( !options_.eccflow.enabled || options_.eccflow.support_scale < 1 ) {
    dst_ecc_image.setTo(1, planetary_disk_mask);
  }
  else {
    /*
     * My current ECC flow implementation produces bugged artifacts when tries to align flat image regions with no gradients.
     * Here is temporary workaround to draw artificial planetary disk with radial intensity gradient from center to edges.
     */
    double min, max;

    cv::distanceTransform(planetary_disk_mask, planetary_disk_mask, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
    cv::minMaxLoc(planetary_disk_mask, &min, &max);
    cv::multiply(planetary_disk_mask, planetary_disk_mask, planetary_disk_mask, 1. / (max * max));

    planetary_disk_mask.copyTo(dst_ecc_image, planetary_disk_mask > FLT_EPSILON);

  }
  return true;
}

bool c_frame_registration::extract_reference_features(cv::InputArray reference_feature_image, cv::InputArray reference_feature_mask)
{
  if( options_.feature_registration.sparse_feature_extractor.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {

    // single planetary disk detection

    const c_feature2d_planetary_disk_detector::options &detector_opts =
        options_.feature_registration.sparse_feature_extractor.detector.planetary_disk_detector;

    if( !detector_opts.align_planetary_disk_masks ) {

      const bool fOk =
          simple_planetary_disk_detector(reference_feature_image,
              reference_feature_mask,
              &planetary_disk_reference_centroid_,
              detector_opts.gbsigma);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

    }
    else {

      const bool fOk =
          simple_planetary_disk_detector(reference_feature_image,
              reference_feature_mask,
              nullptr,
              detector_opts.gbsigma,
              detector_opts.stdev_factor,
              &planetary_disk_reference_component_rect_,
              &planetary_disk_reference_component_mask_,
              &planetary_disk_reference_centroid_);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

      if( !debug_path_.empty() ) {
        save_image(planetary_disk_reference_component_mask_,
            ssprintf("%s/reference_component_mask_.tiff",
                debug_path_.c_str()));
      }
    }

    if( options_.feature_registration.scale != 1 ) {
      planetary_disk_reference_centroid_ /= options_.feature_registration.scale;
    }
  }
  else {

    // generic feature detection

    if( !keypoints_extractor_ && !set_keypoints_extractor(create_keypoints_extractor()) ) {
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

    reference_keypoints_.clear();
    reference_descriptors_.release();

    keypoints_extractor_->detectAndCompute(reference_feature_image, reference_feature_mask,
        reference_keypoints_,
        reference_descriptors_);

    CF_DEBUG("reference_keypoints_.size()=%zu",
        reference_keypoints_.size());

    if( reference_keypoints_.size() < 1 ) {
      CF_ERROR("No reference key points detected : %zu",
          reference_keypoints_.size());
      return false;
    }


    if ( options_.feature_registration.sparse_feature_matcher.type != FEATURE2D_MATCHER_AUTO_SELECT ) {
      keypoints_matcher_ =
          create_sparse_feature_matcher(options_.feature_registration.sparse_feature_matcher);
    }
    else {

      c_feature2d_matcher_options auto_selected_matcher_options =
          options_.feature_registration.sparse_feature_matcher;

      const SPARSE_FEATURE_DESCRIPTOR_TYPE descriptor_type =
          keypoints_extractor_->descriptor_type();

#if HAVE_STAR_EXTRACTOR
      if( descriptor_type == SPARSE_FEATURE_DESCRIPTOR_TRIANGLE ) {
        auto_selected_matcher_options.type = FEATURE2D_MATCHER_TRIANGLES;
      }
#endif
      if( auto_selected_matcher_options.type == FEATURE2D_MATCHER_AUTO_SELECT ) {

        switch (keypoints_extractor_->descriptor()->defaultNorm()) {
        case cv::NORM_HAMMING:
          case cv::NORM_HAMMING2:
          auto_selected_matcher_options.type = FEATURE2D_MATCHER_HAMMING;
          break;
        default:
          auto_selected_matcher_options.type = FEATURE2D_MATCHER_FLANN;
          switch (CV_MAT_DEPTH(keypoints_extractor_->descriptor()->type())) {
          case CV_8U:
            case CV_8S:
            auto_selected_matcher_options.flann.distance_type = cvflann::FLANN_DIST_HAMMING;
            auto_selected_matcher_options.flann.index.type = FlannIndex_lsh;
            break;
          default:
            auto_selected_matcher_options.flann.distance_type = cvflann::FLANN_DIST_L2;
            auto_selected_matcher_options.flann.index.type = FlannIndex_kdtree;
            break;
          }
          break;
        }
      }

      keypoints_matcher_ =
          create_sparse_feature_matcher(auto_selected_matcher_options);
    }

    if( !keypoints_matcher_ ) {
      CF_ERROR("create_sparse_feature_matcher() fails");
      return false;
    }

    keypoints_matcher_->train(reference_descriptors_);
  }


  return true;
}

bool c_frame_registration::estimate_feature_transform(cv::InputArray current_feature_image, cv::InputArray current_feature_mask,
    cv::Mat1f * current_transform)
{
  if( options_.feature_registration.sparse_feature_extractor.detector.type == SPARSE_FEATURE_DETECTOR_PLANETARY_DISK ) {

    // single planetary disk registration

    const c_feature2d_planetary_disk_detector::options &detector_opts =
        options_.feature_registration.sparse_feature_extractor.detector.planetary_disk_detector;

    if( !detector_opts.align_planetary_disk_masks ) {

      const bool fOk =
          simple_planetary_disk_detector(current_feature_image,
              current_feature_mask,
              &planetary_disk_current_centroid_,
              detector_opts.gbsigma);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }
    }
    else {

      const bool fOk =
          simple_planetary_disk_detector(current_feature_image,
              current_feature_mask,
              nullptr,
              detector_opts.gbsigma,
              detector_opts.stdev_factor,
              &planetary_disk_current_component_rect_,
              &planetary_disk_current_component_mask_,
              &planetary_disk_current_centroid_);

      if( !fOk ) {
        CF_FATAL("simple_planetary_disk_detector() fails");
        return false;
      }

      if( !debug_path_.empty() ) {
        save_image(planetary_disk_current_component_mask_,
            ssprintf("%s/current_component_mask_.tiff",
                debug_path_.c_str()));
      }
    }

    if( options_.feature_registration.scale != 1 ) {
      planetary_disk_current_centroid_ /= options_.feature_registration.scale;
    }

    const cv::Point2f current_translation =
        planetary_disk_current_centroid_ - planetary_disk_reference_centroid_;

    *current_transform =
        createTranslationTransform(current_translation.x,
            current_translation.y);

    if( options_.motion_type != ECC_MOTION_TRANSLATION ) {
      *current_transform =
          expandAffineTransform(current_transform_,
              options_.motion_type);
    }
  }
  else {

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
      // TODO: move this code into to estimateTranslation2D()
      if( matched_reference_positions_.size() == 1 ) {
        *current_transform = createTranslationTransform(
            matched_current_positions_[0].x - matched_reference_positions_[0].x,
            matched_current_positions_[0].y - matched_reference_positions_[0].y);
      }
      else if( matched_reference_positions_.size() == 2 ) {
        *current_transform = createTranslationTransform(
            0.5 * (matched_current_positions_[0].x - matched_reference_positions_[0].x + matched_current_positions_[1].x
                - matched_reference_positions_[1].x),
            0.5 * (matched_current_positions_[0].y - matched_reference_positions_[0].y + matched_current_positions_[1].y
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
  }

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

  if ( current_keypoints.size() < 1 ) {
    CF_ERROR("No key points detected : %zu", current_keypoints.size());
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

bool c_frame_registration::base_remap(const cv::Mat2f & rmap,
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

  if ( dst.needed() || dst_mask.needed() ) {
    if ( current_remap_.size() != src_size ) {
      CF_ERROR("WARNING: src size (%dx%d) and remap size (%dx%d) different",
          src_size.width, src_size.height,
          current_remap_.cols, current_remap_.rows);
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



  if( dst.needed() ) {

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

bool c_frame_registration::custom_remap(const cv::Mat2f & rmap,
    cv::InputArray _src, cv::OutputArray dst,
    cv::InputArray _src_mask, cv::OutputArray dst_mask,
    enum ECC_INTERPOLATION_METHOD interpolation_flags,
    enum ECC_BORDER_MODE border_mode,
    const cv::Scalar & border_value) const
{

  bool enable_jovian_derotation =
      options_.jovian_derotation.enabled;

  if( enable_jovian_derotation && rmap.size() != current_remap_.size() ) {
    CF_ERROR("ERROR: Sorry, scaled remaps are not supported for jovian derotations yet.");
    enable_jovian_derotation = false;
  }

  if( !enable_jovian_derotation ) {

    return base_remap(rmap,
        _src, dst,
        _src_mask, dst_mask,
        interpolation_flags,
        border_mode,
        border_value);
  }

  if( dst_mask.needed() ) {

    bool fOk =
        base_remap(rmap,
            cv::noArray(), cv::noArray(),
            _src_mask, dst_mask,
            interpolation_flags,
            border_mode,
            border_value);

    if( !fOk ) {
      CF_ERROR(" c_jovian_rotation_registration:  base::custom_remap() fails");
      return false;
    }
  }

  if( dst.needed() ) {

    cv::Mat2f total_remap =
        rmap.clone();

    const cv::Mat1f & wmask =
        jovian_derotation_.current_cropped_wmask();

    const cv::Rect & rbox =
        jovian_derotation_.reference_bounding_box();

    const cv::Rect & cbox =
        jovian_derotation_.current_bounding_box();


      CF_DEBUG("rmap:  %dx%d", rmap.cols, rmap.rows);
      CF_DEBUG("wmask: %dx%d", wmask.cols, wmask.rows);
      CF_DEBUG("rbox:  x=%d y=%d %dx%d", rbox.x, rbox.y, rbox.width, rbox.height);
      CF_DEBUG("cbox:  x=%d y=%d %dx%d", cbox.x, cbox.y, cbox.width, cbox.height);
      CF_DEBUG("total_remap: %dx%d", total_remap.cols, total_remap.rows);

      const cv::Mat1b bmask =
          (wmask > 0) ;//& (jovian_derotation_.current_uncropped_planetary_disk_mask()(rbox));

    cv::Mat2f derotation_remap;

    cv::add(jovian_derotation_.current_cropped_derotation_remap(),
        cv::Scalar(cbox.x, cbox.y),
        derotation_remap);

    derotation_remap.copyTo(total_remap(rbox),
        bmask);


    bool fOk =
        base_remap(total_remap,
            _src, dst,
            cv::noArray(), cv::noArray(),
            interpolation_flags,
            border_mode,
            border_value);

    if( !fOk ) {
      CF_ERROR(" c_jovian_rotation_registration:  base::custom_remap() fails fails");
      return false;
    }
  }
//
  if( dst_mask.needed() ) {

    const cv::Mat orig_mask =
        dst_mask.getMat();

    CF_DEBUG("dst_mask: %dx%d channels=%d depth=%d",
        dst_mask.cols(), dst_mask.rows(),
        dst_mask.channels(),
        dst_mask.depth());

    cv::Mat new_mask;
    if ( orig_mask.depth() == CV_32F ) {
      orig_mask.copyTo(new_mask);
    }
    else {
      orig_mask.convertTo(new_mask, CV_32F, 1./255);
    }


    const cv::Mat1f & wmask =
        jovian_derotation_.current_cropped_wmask();


    new_mask.setTo(0, jovian_derotation_.current_uncropped_planetary_disk_mask());
    wmask.copyTo(new_mask(jovian_derotation_.reference_bounding_box()), wmask > 0);
    //cv::GaussianBlur(new_mask, new_mask, cv::Size(), 2, 2);
    new_mask.setTo(0, ~orig_mask);
    dst_mask.move(new_mask);
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

