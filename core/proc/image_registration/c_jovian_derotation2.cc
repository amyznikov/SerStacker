/*
 * c_jovian_derotation2.cc
 *
 *  Created on: Aug 28, 2024
 *      Author: amyznikov
 */

#include "c_jovian_derotation2.h"
#include <core/proc/ellipsoid.h>
#include <core/proc/pose.h>
#include <core/debug.h>

bool c_jovian_derotation2::detect(cv::InputArray reference_image, cv::InputArray reference_mask)
{
  INSTRUMENT_REGION("");
//
//  _reference_image_size =
//      reference_image.size();

  if( !_detector.detect(reference_image, reference_mask) ) {
    CF_ERROR("saturn_detector_.detect() fails");
    return false;
  }

  return true;
}


bool c_jovian_derotation2::compute(double current_tstamp_sec, double target_tstamp_sec)
{
  return _detector.compute_derotation_for_time(current_tstamp_sec - target_tstamp_sec);
//
//  // Jupiter daily rotation period is 9h 55m 30s.
//
//  static constexpr double rotation_period_sec =
//      9. * 3660 + 55. * 60 + 30.;
//
//  const double rotation_angle_deg =
//      360 * (current_tstamp_sec - target_tstamp_sec) / rotation_period_sec;
//
//  return compute(rotation_angle_deg);
}


bool c_jovian_derotation2::compute(double zrotation_deg)
{
  return _detector.compute_derotation_for_angle(zrotation_deg);
}

