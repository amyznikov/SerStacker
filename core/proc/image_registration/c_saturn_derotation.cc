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
  _detector.options() = v;
}

const c_saturn_ellipse_detector_options & c_saturn_derotation::detector_options() const
{
  return _detector.options();
}

bool c_saturn_derotation::detect(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");

  _reference_image_size =
      reference_image.size();

  if( !_detector.detect(reference_image, reference_mask) ) {
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


bool c_saturn_derotation::compute(double zrotation_deg)
{
  cv::Mat2f rmap;
  cv::Mat1b rmask;
  cv::Mat remapped_image;

  compute_ellipsoid_zrotation_remap(_reference_image_size, _detector.center(),
      _detector.axes(),
      _detector.pose(),
      zrotation_deg * CV_PI / 180,
      rmap, rmask);

  //  cv::remap(image.getMat(), remapped_image, rmap, cv::noArray(), cv::INTER_LINEAR,
  //      cv::BORDER_CONSTANT, cv::Scalar::all(0));
  //  remapped_image.copyTo(image, rmask);

  return true;
}

bool c_saturn_derotation::compute(double current_tstamp_sec, double target_tstamp_sec)
{
  // Saturn daily rotation period is 10h 33m 38s.

  static constexpr double rotation_period_sec =
      10. * 3660 + 33. * 60 + 38.;

  const double rotation_angle_deg =
      360 * (current_tstamp_sec - target_tstamp_sec) / rotation_period_sec;

  return compute(rotation_angle_deg);
}

