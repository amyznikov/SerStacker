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


class c_triangle_extractor :
  public cv::Feature2D
{
public:
  typedef c_triangle_extractor this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;

  c_triangle_extractor(int min_side_size = 10);

  static cv::Ptr<c_triangle_extractor> create(int min_side_size = 10);

  void compute( cv::InputArray /*image*/,
      CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors ) override;

  int descriptorSize() const override;
  int descriptorType() const override;
  int defaultNorm() const override;

protected:
  int min_side_size_ = 10;
};


/**
 * Options for c_sorted_norm_based_feature2d_matcher
 */
struct c_triangle_matcher_options :
    c_feature2d_matcher_base_options
{
   double eps = 1e-4;
};


class c_triangle_matcher :
  public c_feature2d_matcher
{
public:
  typedef c_triangle_matcher this_class;
  typedef c_feature2d_matcher base;
  typedef cv::Ptr<this_class> ptr;
  typedef cv::flann::L2_Simple<float> DistanceType;

  c_triangle_matcher(double eps);

  static ptr create(double eps);

  bool train(const std::vector<cv::KeyPoint> * train_keypoints, cv::InputArray train_descriptors) override;
  bool match(const std::vector<cv::KeyPoint> * query_keypoints, cv::InputArray query_descriptors,
      /* out */ std::vector<cv::DMatch> & matches) override;

protected:
  cv::Mat1b reference_triangles_;
  cvflann::Matrix<DistanceType::ElementType> reference_features_;
  cv::Ptr<cvflann::KDTreeIndex<DistanceType>> index_;
  double eps_ = 1e-4;
};

c_triangle_matcher::ptr create_sparse_feature_matcher(
    const c_triangle_matcher_options & options);


#endif /* __c_triangle_matcher_h__ */
