/*
 * c_planetary_disk_matcher.cc
 *
 *  Created on: Jul 12, 2026
 *      Author: amyznikov
 */

#include "c_planetary_disk_matcher.h"

c_planetary_disk_matcher::c_planetary_disk_matcher()
{
}

c_planetary_disk_matcher::c_planetary_disk_matcher(const c_planetary_disk_matcher_options & opts) :
    _opts(opts)
{
}

c_planetary_disk_matcher::ptr c_planetary_disk_matcher::create(const c_planetary_disk_matcher_options & opts)
{
  return ptr(new this_class(opts));
}

bool c_planetary_disk_matcher::train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray train_descriptors)
{
  if ( !train_keypoints.empty() ) {
    _reference_planetary_disk_postion = train_keypoints.front().pt;
    _reference_planetary_disk_size.width = _reference_planetary_disk_size.height = train_keypoints.front().size;
    return true;
  }

  if ( !train_descriptors.empty() ) {
    if ( train_descriptors.type() == CV_32FC1 && train_descriptors.cols() == 4 ) {
      const cv::Mat1f M = train_descriptors.getMat();
      const float * mp = M[0];
      _reference_planetary_disk_postion.x = mp[0];
      _reference_planetary_disk_postion.y = mp[1];
      _reference_planetary_disk_size.width = mp[2];
      _reference_planetary_disk_size.height = mp[2];
      return true;
    }
  }

  CF_ERROR("Invalid planetary_disk descriptor");
  return false;
}

bool c_planetary_disk_matcher::match(const std::vector<cv::KeyPoint> & query_keypoints, cv::InputArray query_descriptors,
    /* out */ std::vector<cv::DMatch> & matches)
{
  //  cv::Point2f currentPos;
  //  cv::Size currentSize;

  if ( !query_keypoints.empty() ) {
    //  currentPos = query_keypoints.front().pt;
    //  currentSize.width = currentSize.height = query_keypoints.front().size();
    matches.emplace_back(0, 0, 1);
    return true;
  }

  if ( !query_descriptors.empty() ) {
    if ( query_descriptors.type() == CV_32FC1 && query_descriptors.cols() == 4 ) {
      //  const cv::Mat1f M = query_descriptors.getMat();
      //  const float * mp = M[0];
      //  currentPos.x = mp[0];
      //  currentPos.y = mp[1];
      //  currentSize.width = mp[2];
      //  currentSize.height = mp[2];
      matches.emplace_back(0, 0, 1);
      return true;
    }
  }

  CF_ERROR("Invalid planetary_disk descriptor");
  return false;
}

c_planetary_disk_matcher::ptr create_sparse_feature_matcher(const c_planetary_disk_matcher_options & opts)
{
  return c_planetary_disk_matcher::create(opts);
}
