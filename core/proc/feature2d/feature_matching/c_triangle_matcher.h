/*
 * c_triangle_matcher.h
 *
 *  Created on: Jun 2, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_triangle_matcher_h__
#define __c_triangle_matcher_h__

#include "c_feature2d_matcher.h"
#include <opencv2/flann.hpp>
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>

struct c_triangle_extractor_options :
    c_feature2d_matcher_base_options
{
  int max_points = 20;
  int min_side_size = 20;
};

bool serialize_triangle_extractor_options(c_config_setting section, bool save,
    c_triangle_extractor_options & opts);

inline bool save_settings(c_config_setting section, const c_triangle_extractor_options & opts)
{
  return serialize_triangle_extractor_options(section, true,
      const_cast<c_triangle_extractor_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_triangle_extractor_options * opts)
{
  return serialize_triangle_extractor_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_triangle_extractor_options> & ctx)
{
  using S = c_triangle_extractor_options;
  ctlbind(ctls, "max_points", ctx(&S::max_points), "");
  ctlbind(ctls, "min_side_size [px]", ctx(&S::min_side_size), "");
}

class c_triangle_extractor :
  public cv::Feature2D
{
public:
  typedef c_triangle_extractor this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;

  c_triangle_extractor();
  explicit c_triangle_extractor(const c_triangle_extractor_options & opts);

  static cv::Ptr<c_triangle_extractor> create();
  static cv::Ptr<c_triangle_extractor> create(const c_triangle_extractor_options & opts);

  void set_options(const c_triangle_extractor_options & opts)
  {
    _opts = opts;
  }

  const c_triangle_extractor_options & options() const
  {
    return _opts;
  }

  void compute( cv::InputArray /*image*/,
      CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors) override;

  int descriptorSize() const override;
  int descriptorType() const override;
  int defaultNorm() const override;

protected:
  c_triangle_extractor_options _opts;
};


/**
 * Options for c_sorted_norm_based_feature2d_matcher
 */
struct c_triangle_matcher_options :
    c_feature2d_matcher_base_options
{
   double eps = 1e-4;
};

bool serialize_triangle_matcher_options(c_config_setting section, bool save,
    c_triangle_matcher_options & opts);

inline bool save_settings(c_config_setting section, const c_triangle_matcher_options & opts)
{
  return serialize_triangle_matcher_options(section, true,
      const_cast<c_triangle_matcher_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_triangle_matcher_options * opts)
{
  return serialize_triangle_matcher_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_triangle_matcher_options> & ctx)
{
  using S = c_triangle_matcher_options;
  ctlbind(ctls, "eps", ctx(&S::eps), "");
}

class c_triangle_matcher :
  public c_feature2d_matcher
{
public:
  typedef c_triangle_matcher this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;
  typedef cv::flann::L2_Simple<float> DistanceType;

  c_triangle_matcher();
  explicit c_triangle_matcher(const c_triangle_matcher_options & opts);

  static ptr create();
  static ptr create(const c_triangle_matcher_options & opts);

  void set_options(const c_triangle_matcher_options & opts)
  {
    _opts = opts;
  }

  const c_triangle_matcher_options & options() const
  {
    return _opts;
  }

  bool train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray train_descriptors) override;
  bool match(const std::vector<cv::KeyPoint> & query_keypoints, cv::InputArray query_descriptors,
      /* out */ std::vector<cv::DMatch> & matches) override;

protected:
  cv::Mat1b _reference_triangles;
  cvflann::Matrix<DistanceType::ElementType> _reference_features;
  cv::Ptr<cvflann::KDTreeIndex<DistanceType>> _index;
  c_triangle_matcher_options _opts;
};

c_triangle_matcher::ptr create_sparse_feature_matcher(
    const c_triangle_matcher_options & options);


#endif /* __c_triangle_matcher_h__ */
