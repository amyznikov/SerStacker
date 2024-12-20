/*
 * c_simple_planetary_disk_detector.cc
 *
 *  Created on: Jun 5, 2022
 *      Author: amyznikov
 */

#include "c_simple_planetary_disk_detector.h"
#include <core/proc/planetary-disk-detection.h>
#include <core/debug.h>

c_simple_planetary_disk_detector::c_simple_planetary_disk_detector(double gbsigma, double stdev_factor, int se_close_radius) :
  gbsigma_(gbsigma),
  stdev_factor_(stdev_factor),
  se_close_radius_(se_close_radius_)
{
}

cv::Ptr<c_simple_planetary_disk_detector> c_simple_planetary_disk_detector::create(double gbsigma, double stdev_factor, int se_close_radius)
{
  return cv::Ptr<this_class>(new c_simple_planetary_disk_detector(gbsigma, stdev_factor, se_close_radius));
}

void c_simple_planetary_disk_detector::detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints,
    cv::InputArray _mask)
{
  cv::Point2f centrold;

  bool fOk =
      simple_planetary_disk_detector(
          _src,
          _mask,
          gbsigma_,
          stdev_factor_,
          se_close_radius_,
          &centrold,
          &component_rect_,
          nullptr/* &cmponent_mask_*/,
          nullptr/* &geometrical_center_*/,
          nullptr/* &debug_image*/);

  if( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails. Can not detect planetary disk on given image");
  }
  else {
    //float angle=-1, float response=0, int octave=0, int class_id=-1
    keypoints.emplace_back(cv::KeyPoint(centrold, std::max(component_rect_.width, component_rect_.height)));
  }
}

