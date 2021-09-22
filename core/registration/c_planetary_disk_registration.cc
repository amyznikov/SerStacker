/*
 * c_planetary_disk_image_registration.cc
 *
 *  Created on: Aug 28, 2020
 *      Author: amyznikov
 */

#include "c_planetary_disk_registration.h"
#include <core/proc/planetary-disk-detection.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>


c_planetary_disk_registration::c_planetary_disk_registration()
{
}

c_planetary_disk_registration::c_planetary_disk_registration(const c_planetary_disk_registration_options & opts)
  : options_(opts)
{
}

c_planetary_disk_registration::c_planetary_disk_registration(const c_frame_registration_base_options & base_opts, const c_planetary_disk_registration_options & opts)
  : base(base_opts), options_(opts)
{
}

c_planetary_disk_registration::ptr c_planetary_disk_registration::create()
{
  return ptr(new c_planetary_disk_registration());
}

c_planetary_disk_registration::ptr c_planetary_disk_registration::create(const c_planetary_disk_registration_options & opts)
{
  return ptr(new c_planetary_disk_registration(opts));
}

c_planetary_disk_registration::ptr c_planetary_disk_registration::create(const c_frame_registration_base_options & base_opts, const c_planetary_disk_registration_options & opts)
{
  return ptr(new c_planetary_disk_registration(base_opts, opts));
}


c_planetary_disk_registration_options & c_planetary_disk_registration::planetary_disk_registration_options()
{
  return options_;
}

const c_planetary_disk_registration_options & c_planetary_disk_registration::planetary_disk_registration_options() const
{
  return options_;
}

bool c_planetary_disk_registration::create_feature_image(cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray dst, cv::OutputArray dstmask) const
{
  if ( !extract_channel(src, dst, srcmask, dstmask, registration_channel(), feature_scale()) ) {
    CF_ERROR("extract_channel(registration_channel__=%d) fails", registration_channel());
    return false;
  }

  return true;
}

bool c_planetary_disk_registration::extract_reference_features(cv::InputArray feature_image,
    cv::InputArray feature_mask)
{
  double gbsigma = 0.5;

  if ( !options_.align_planetary_disk_masks ) {
    if ( !simple_planetary_disk_detector(feature_image, feature_mask, &reference_centroid_, gbsigma) ) {
      CF_FATAL("simple_small_planetary_disk_detector() fails");
      return false;
    }
  }
  else {
    if ( !simple_planetary_disk_detector(feature_image, feature_mask, nullptr, gbsigma,
        &reference_component_rect_, &reference_component_mask_, &reference_centroid_) ) {
      CF_FATAL("simple_small_planetary_disk_detector() fails");
      return false;
    }

    if ( enable_debug_ && !debug_path_.empty() ) {
      save_image(reference_component_mask_,
          ssprintf("%s/reference_component_mask_.tiff",
              debug_path_.c_str()));
    }

  }

  if ( feature_scale() != 1.0 && feature_scale() != 0 ) {
    reference_centroid_ /= feature_scale();
  }

  return true;
}

bool c_planetary_disk_registration::create_reference_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  if ( !options_.align_planetary_disk_masks ) {
    return create_ecc_image(src, srcmsk, dst, dstmsk, scale);
  }

  if ( scale <= 0 || scale == 1 ) {
    reference_component_mask_.convertTo(dst, CV_32F, 1. / 255);
    srcmsk.copyTo(dstmsk);
  }
  else if ( !extract_channel(reference_component_mask_, dst, srcmsk, dstmsk, 0, scale, CV_32F, 1. / 255) ) {
    CF_ERROR("extract_channel() fails");
    return false;
  }

  return true;
}

bool c_planetary_disk_registration::create_current_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  if ( !options_.align_planetary_disk_masks ) {
    return create_ecc_image(src, srcmsk, dst, dstmsk, scale);
  }

  if ( scale <= 0 || scale == 1 ) {
    current_component_mask_.convertTo(dst, CV_32F, 1. / 255);
    srcmsk.copyTo(dstmsk);
  }
  else if ( !extract_channel(current_component_mask_, dst, srcmsk, dstmsk, 0, scale, CV_32F, 1. / 255) ) {
    CF_ERROR("extract_channel() fails");
    return false;
  }

  return true;
}


bool c_planetary_disk_registration::estimate_feature_transform(cv::InputArray feature_image,
    cv::InputArray feature_mask,
    cv::Mat1f * current_transform)
{
  double gbsigma = 0.5;

  if ( !options_.align_planetary_disk_masks ) {
    if ( !simple_planetary_disk_detector(feature_image, feature_mask, &current_centroid_, gbsigma) ) {
      CF_FATAL("simple_small_planetary_disk_detector() fails");
      return false;
    }
  }
  else {
    if ( !simple_planetary_disk_detector(feature_image, feature_mask, nullptr, gbsigma,
        &current_component_rect_, &current_component_mask_, &current_centroid_) ) {
      CF_FATAL("simple_small_planetary_disk_detector() fails");
      return false;
    }

    if ( enable_debug_ && !debug_path_.empty() ) {
      save_image(current_component_mask_,
          ssprintf("%s/current_component_mask_.tiff",
              debug_path_.c_str()));
    }
  }

  if ( feature_scale() != 1.0 && feature_scale() != 0 ) {
    current_centroid_ /= feature_scale();
  }

  const cv::Point2f current_translation =
      current_centroid_ - reference_centroid_;

  *current_transform = createTranslationTransform(current_translation.x, current_translation.y);
  if ( motion_type() != ECC_MOTION_TRANSLATION ) {
    *current_transform = expandAffineTransform(current_transform_, motion_type());
  }

  return true;
}

const cv::Point2f & c_planetary_disk_registration::current_centroid() const
{
  return current_centroid_;
}

const cv::Point2f & c_planetary_disk_registration::reference_centroid() const
{
  return reference_centroid_;
}
