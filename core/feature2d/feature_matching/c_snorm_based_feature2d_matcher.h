/*
 * c_sorted_norm_based_feature2d_matcher.h
 *
 *  Created on: Jan 6, 2022
 *      Author: amyznikov
 *
 *  This VERY experiental code is NOT reccomended to use in practice.
 *  For SIFT/SURF descriptors prefer the FLANN_INDEX_KDTREE instead.
 *
 */

#pragma once
#ifndef __c_sorted_norm_based_feature2d_matcher_h__
#define __c_sorted_norm_based_feature2d_matcher_h__

#include "c_feature2d_matcher.h"

/**
 * Options for c_sorted_norm_based_feature2d_matcher
 */
struct c_snorm_based_feature2d_matcher_options :
    c_feature2d_matcher_base_options
{
  float max_acceptable_distance = -1; // auto select
  float lowe_ratio = -1;
};


/** @brief Sparse feature2d descriptor matcher based on sorted norm of feature descriptors
 */
class c_snorm_based_feature2d_matcher :
  public c_feature2d_matcher
{
public:
  typedef c_snorm_based_feature2d_matcher this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;

public:
  c_snorm_based_feature2d_matcher();

  void set_max_acceptable_distance(double v);
  double max_acceptable_distance() const;

  void set_lowe_ratio(double v);
  double lowe_ratio() const;

  bool train( cv::InputArray train_descriptors) override;
  bool match(cv::InputArray query_descriptors, /* out */ std::vector<cv::DMatch> & matches) override;

protected:
  struct index_entry {
    int row;
    float norm;
    index_entry(int r, float n) :
      row(r), norm(n)
    {}
  };

  cv::Mat1f train_descriptors_;
  cv::Mat1f query_descriptors_;
  std::vector<index_entry> index_;
  float max_acceptable_distance_ = -1; // auto select
  float lowe_ratio_ = -1;
};


c_snorm_based_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_snorm_based_feature2d_matcher_options & options);


#endif /* __c_sorted_norm_based_feature2d_matcher_h__ */
