/*
 * c_star_extractor.cc
 *
 *  Created on: Jun 2, 2022
 *      Author: amyznikov
 */

#include "c_star_extractor.h"
#include <core/debug.h>

c_star_extractor::c_star_extractor()
{
}

cv::Ptr<c_star_extractor> c_star_extractor::create()
{
  return cv::Ptr<this_class>(new this_class());
}

cv::Ptr<c_star_extractor> c_star_extractor::create(const c_simple_star_detector_options & opts)
{
  cv::Ptr<this_class> obj(new this_class());
  obj->_detector.set_options(opts);
  return obj;
}

void c_star_extractor::set_opts(const c_simple_star_detector_options & opts)
{
  _detector.set_options(opts);
}

const c_simple_star_detector_options & c_star_extractor::opts() const
{
  return _detector.options();
}

void c_star_extractor::detect(cv::InputArray src, std::vector<cv::KeyPoint> & keypoints, cv::InputArray mask)
{
  const auto & blobs = _detector.detect(src, mask);

  keypoints.clear(), keypoints.reserve(blobs.size());

  for ( const auto & blob: blobs ) {
    keypoints.emplace_back(cv::Point2f(blob.x, blob.y), 5 * blob.a, blob.theta * 180 / CV_PI, blob.I);
  }
}
