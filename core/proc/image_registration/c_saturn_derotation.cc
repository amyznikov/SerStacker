/*
 * c_saturn_derotation.cc
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#include "c_saturn_derotation.h"
#include <core/proc/ellipsoid.h>
#include <core/proc/pose.h>
#include <core/debug.h>

void c_saturn_derotation::set_detector_options(const c_saturn_ellipse_detector_options & v)
{
  saturn_detector_.options() = v;
}

const c_saturn_ellipse_detector_options & c_saturn_derotation::detector_options() const
{
  return saturn_detector_.options();
}

bool c_saturn_derotation::detect_saturn(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  reference_image_size_ =
      reference_image.size();

  if( !saturn_detector_.detect(reference_image, reference_mask) ) {
    CF_ERROR("saturn_detector_.detect() fails");
    return false;
  }


  //  jovian_ellipse_mask_ = saturn_detector_.final_planetary_disk_mask();
  //  jovian_ellipse_ = saturn_detector_.final_planetary_disk_ellipse();
  //  jovian_crop_ = compute_ellipse_crop_box(jovian_ellipse_, reference_image.size());

//  CF_DEBUG("jovian_crop_: x=%d y=%d w=%d h=%d",
//      jovian_crop_.x, jovian_crop_.y,
//      jovian_crop_.width, jovian_crop_.height);
//
//  return false;
//

  return true;

}

bool c_saturn_derotation::compute(double zrotation)
{
  cv::Mat2f rmap;
  cv::Mat1b rmask;
  cv::Mat remapped_image;

  compute_ellipsoid_zrotation_remap(reference_image_size_, saturn_detector_.center(),
      saturn_detector_.ellipsoid_size(),
      saturn_detector_.ellipsoid_rotation(),
      zrotation * CV_PI / 180,
      rmap, rmask);

  //  cv::remap(image.getMat(), remapped_image, rmap, cv::noArray(), cv::INTER_LINEAR,
  //      cv::BORDER_CONSTANT, cv::Scalar::all(0));
  //  remapped_image.copyTo(image, rmask);

  return true;
}

