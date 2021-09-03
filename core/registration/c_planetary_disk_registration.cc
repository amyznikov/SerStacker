/*
 * c_planetary_disk_image_registration.cc
 *
 *  Created on: Aug 28, 2020
 *      Author: amyznikov
 */

#include "c_planetary_disk_registration.h"
#include <core/proc/planetary-disk-detector.h>
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
  return c_planetary_disk_registration::ptr(new c_planetary_disk_registration());
}

c_planetary_disk_registration::ptr c_planetary_disk_registration::create(const c_planetary_disk_registration_options & opts)
{
  return c_planetary_disk_registration::ptr(new c_planetary_disk_registration(opts));
}

c_planetary_disk_registration::ptr c_planetary_disk_registration::create(const c_frame_registration_base_options & base_opts, const c_planetary_disk_registration_options & opts)
{
  return c_planetary_disk_registration::ptr(new c_planetary_disk_registration(base_opts, opts));
}


c_planetary_disk_registration_options & c_planetary_disk_registration::options()
{
  return options_;
}

const c_planetary_disk_registration_options & c_planetary_disk_registration::options() const
{
  return options_;
}

//bool c_planetary_disk_registration::select_crop_rectangle(const cv::Size & image_size, const cv::Size & crop_size,
//    const cv::Point2f & crop_center, cv::Rect * ROI) const
//{
//  if ( !crop_size.empty() ) {
//
//    if ( image_size.width < crop_size.width || image_size.height < crop_size.height ) {
//      CF_ERROR("ERROR: crop size = %dx%d larger than frame size = %dx%d in planetary_disk_registration",
//          crop_size.width, crop_size.height, image_size.width, image_size.height);
//      return false;
//    }
//
//    int l = cv::max(0, (int) crop_center.x - crop_size.width / 2);
//    int t = cv::max(0, (int) crop_center.y - crop_size.height / 2);
//    if ( l + crop_size.width > image_size.width ) {
//      l = image_size.width - crop_size.width;
//    }
//    if ( t + crop_size.height > image_size.height ) {
//      t = image_size.height - crop_size.height;
//    }
//
//    ROI->x = l;
//    ROI->y = t;
//    ROI->width = crop_size.width;
//    ROI->height = crop_size.height;
//  }
//
//  return true;
//}


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
  if ( !simple_planetary_disk_detector(feature_image, feature_mask, &reference_centroid_) ) {
    CF_FATAL("simple_small_planetary_disk_detector() fails");
    return false;
  }

  if ( feature_scale() != 1.0 && feature_scale() != 0 ) {
    reference_centroid_ /= feature_scale();
  }

//  if ( !options_.crop_size.empty() && options_.crop_size != current_input_frame_size_ ) {
//
//    if ( !select_crop_rectangle(current_input_frame_size_, options_.crop_size, reference_centroid_, &reference_ROI_) ) {
//      CF_FATAL("select_crop_rectangle() fails");
//      return false;
//    }
//
//    reference_centroid_.x -= reference_ROI_.x;
//    reference_centroid_.y -= reference_ROI_.y;
//  }



  return true;
}

bool c_planetary_disk_registration::create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
    cv::OutputArray dst, cv::OutputArray dstmsk,
    double scale) const
{
  return base::create_ecc_image(src, srcmsk, dst, dstmsk, scale);
}

bool c_planetary_disk_registration::estimate_feature_transform(cv::InputArray feature_image,
    cv::InputArray feature_mask,
    cv::Mat1f * current_transform)
{
  if ( !simple_planetary_disk_detector(feature_image, feature_mask, &current_centroid_) ) {
    CF_FATAL("simple_small_planetary_disk_detector() fails");
    return false;
  }

  if ( feature_scale() != 1.0 && feature_scale() != 0 ) {
    current_centroid_ /= feature_scale();
  }

//  if ( !options_.crop_size.empty() && options_.crop_size != current_input_frame_size_ ) {
//
//    if ( !select_crop_rectangle(current_input_frame_size_, options_.crop_size, current_centroid_, &current_ROI_) ) {
//      CF_FATAL("select_crop_rectangle() fails");
//      return false;
//    }
//
//    current_centroid_.x -= current_ROI_.x;
//    current_centroid_.y -= current_ROI_.y;
//  }

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
