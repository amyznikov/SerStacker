/*
 * c_hamming_distance_feature2d_matcher.h
 *
 *  Created on: Jan 4, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hamming_distance_feature2d_matcher_h__
#define __c_hamming_distance_feature2d_matcher_h__

#include "c_feature2d_matcher.h"

/**
 * Options for c_hamming_distance_feature2d_matcher
 */
struct c_hamming_distance_feature2d_matcher_options :
    c_feature2d_matcher_base_options
{
  int max_acceptable_distance = -1; // auto select
  int octavedif = -1; // ignore by default
};

/** @brief Sparse feature2d descriptor matcher based on haming distance
 */
class c_hamming_distance_feature2d_matcher :
  public c_feature2d_matcher
{
public:
  typedef c_hamming_distance_feature2d_matcher this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;

  c_hamming_distance_feature2d_matcher() = default;

  void set_max_acceptable_distance(int v);
  int max_acceptable_distance() const;

  bool train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray train_descriptors) override;
  bool match(const std::vector<cv::KeyPoint> & query_keypoints, cv::InputArray query_descriptors,
      /* out */ std::vector<cv::DMatch> & matches) override;

protected:
  struct index_entry {
    uint16_t row, norm, octave;
    index_entry(uint16_t r, uint16_t n, uint16_t oct) :
      row(r), norm(n), octave(oct)
    {}
  };

  cv::Mat1i _train_descriptors;
  std::vector<index_entry> _index;
  int _max_acceptable_distance = -1; // auto select
};


c_hamming_distance_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_hamming_distance_feature2d_matcher_options & options);


#endif /* __c_hamming_distance_feature2d_matcher_h__ */
