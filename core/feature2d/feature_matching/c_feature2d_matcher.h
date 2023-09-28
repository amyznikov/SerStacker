/*
 * c_feature2d_matcher.h
 *
 *  Created on: Feb 8, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_feature2d_matcher_h__
#define __c_feature2d_matcher_h__

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

/* Base options for sparse feature2d matchers */
struct c_feature2d_matcher_base_options
{
};


/** @brief Sparse feature2d descriptor matcher abstract base class
 */
class c_feature2d_matcher
{
public:
  typedef c_feature2d_matcher this_class;
  typedef cv::Ptr<this_class> sptr;

  c_feature2d_matcher(int type = -1)  :
    type_(type)
  {
  }

  void set_type(int v)
  {
    type_ = v;
  }

  int type() const
  {
    return type_;
  }

  virtual ~c_feature2d_matcher() = default;
  virtual bool train(const std::vector<cv::KeyPoint> * train_keypoints, cv::InputArray train_descriptors) = 0;
  virtual bool match(const std::vector<cv::KeyPoint> * query_keypoints, cv::InputArray query_descriptors,
      /*out*/ std::vector<cv::DMatch> & output_matches) = 0;

protected:
  int type_ = -1;
};


#endif /* __c_feature2d_matcher_h__ */
