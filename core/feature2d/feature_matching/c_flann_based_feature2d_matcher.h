/*
 * c_flann_based_feature2d_matcher.h
 *
 *  Created on: Jan 4, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_flann_based_feature2d_matcher_h__
#define __c_flann_based_feature2d_matcher_h__

#include "c_feature2d_matcher.h"
#include <core/ssprintf.h>

#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) \
    ((a)<<16 | (b)<<8 | (c))
#endif

#ifndef CV_VERSION_CURRRENT
#define CV_VERSION_CURRRENT \
    CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif


enum FlannIndexType {
  FlannIndex_unknown  = -1,
  FlannIndex_linear = 0,
  FlannIndex_kdtree = 1,
  FlannIndex_kmeans = 2,
  FlannIndex_composite = 3,
  FlannIndex_hierarchical = 4,
  FlannIndex_lsh = 5,
  FlannIndex_autotuned = 6,
};



struct c_flann_linear_index_options
{
};

struct c_flann_kdtree_index_options
{
  int trees = 1;
};

struct c_flann_kmeans_index_options
{
  cvflann::flann_centers_init_t centers_init =
      cvflann::FLANN_CENTERS_RANDOM;
  int branching = 32;
  int iterations = 11;
  float cb_index = 0.2f;
};

struct c_flann_composite_index_options
{
  cvflann::flann_centers_init_t centers_init =
      cvflann::FLANN_CENTERS_RANDOM;
  int trees = 1;
  int branching = 32;
  int iterations = 11;
  float cb_index = 0.2f;
};

struct c_flann_hierarchical_index_options
{
  cvflann::flann_centers_init_t centers_init =
      cvflann::FLANN_CENTERS_RANDOM;
  int branching = 32;
  int trees = 4;
  int leaf_size = 100;
};

struct c_flann_lsh_index_options
{
  int table_number = 8;
  int key_size = 12;
  int multi_probe_level = 1;
};

struct c_flann_autotuned_index_options
{
  float target_precision = 0.8f;
  float build_weight = 0.01f;
  float memory_weight = 0;
  float sample_fraction = 0.1f;
};


struct c_flann_index_options
{
  FlannIndexType type = FlannIndex_kdtree;
  c_flann_linear_index_options linear;
  c_flann_kdtree_index_options kdtree;
  c_flann_kmeans_index_options kmeans;
  c_flann_composite_index_options composite;
  c_flann_hierarchical_index_options hierarchical;
  c_flann_lsh_index_options lsh;
  c_flann_autotuned_index_options autotuned;
};

/**
 * Options for c_flann_based_feature2d_matcher
 * */
struct c_flann_based_feature2d_matcher_options : c_feature2d_matcher_base_options
{
  cvflann::flann_distance_t distance_type =
      cvflann::FLANN_DIST_L2;

  c_flann_index_options index;

  double lowe_ratio = -1;
};

/**
 * cv::FLANN-based matcher
 */
class c_flann_based_feature2d_matcher :
    public c_feature2d_matcher
{
public:
  typedef c_flann_based_feature2d_matcher this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;

  c_flann_based_feature2d_matcher(const cv::Ptr<cv::flann::IndexParams> & index_args);

  void set_distance_type(cvflann::flann_distance_t v);
  cvflann::flann_distance_t distance_type() const;

  void set_lowe_ratio(double v);
  double lowe_ratio() const;

  const cv::Ptr<cv::flann::IndexParams> & index_params() const;

  bool train( cv::InputArray train_descriptors) override;
  bool match(cv::InputArray query_descriptors, /* out */ std::vector<cv::DMatch> & matches) override;

protected:
  double lowe_ratio_ = -1;
  cvflann::flann_distance_t distance_type_ = cvflann::FLANN_DIST_L2;
  cv::flann::Index index_;
  cv::Ptr<cv::flann::IndexParams> index_params_;
  cv::Ptr<cv::flann::SearchParams> search_params_;
  cv::Mat1i indices_;
  cv::Mat dists_;
};

template<>
const c_enum_member *
  members_of<cvflann::flann_centers_init_t>();

template<>
const c_enum_member *
  members_of<cvflann::flann_distance_t>();

template<>
const c_enum_member *
  members_of<FlannIndexType>();


c_flann_based_feature2d_matcher::ptr create_sparse_feature_matcher(
    const c_flann_based_feature2d_matcher_options & options);

#endif /* __c_flann_based_feature2d_matcher_h__ */
