/*
 * c_planetary_disk_matcher.h
 *
 *  Created on: Jul 12, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_planetary_disk_matcher_h__
#define __c_planetary_disk_matcher_h__

#include "c_feature2d_matcher.h"

/**
 * Options for c_sorted_norm_based_feature2d_matcher
 */
struct c_planetary_disk_matcher_options :
    c_feature2d_matcher_base_options
{
};


class c_planetary_disk_matcher :
      public c_feature2d_matcher
{
public:
  typedef c_planetary_disk_matcher this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;

  c_planetary_disk_matcher();
  c_planetary_disk_matcher(const c_planetary_disk_matcher_options & opts);

  static ptr create(const c_planetary_disk_matcher_options & opts);

  bool train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray train_descriptors) override;
  bool match(const std::vector<cv::KeyPoint> & query_keypoints, cv::InputArray query_descriptors,
      /* out */ std::vector<cv::DMatch> & matches) override;

protected:
  c_planetary_disk_matcher_options _opts;
  cv::Point2f _reference_planetary_disk_postion;
  cv::Size _reference_planetary_disk_size;
};

c_planetary_disk_matcher::ptr create_sparse_feature_matcher(const c_planetary_disk_matcher_options & opts);

#endif /* __c_planetary_disk_matcher_h__ */
