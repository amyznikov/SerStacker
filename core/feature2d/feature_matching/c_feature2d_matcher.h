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
struct c_feature2d_matcher_base_options {
};


/** @brief Sparse feature2d descriptor matcher abstract base class
 */
class c_feature2d_matcher
{
public:
  typedef c_feature2d_matcher this_class;
  typedef cv::Ptr<this_class> ptr;

  c_feature2d_matcher() = default;
  virtual ~c_feature2d_matcher() = default;

  virtual bool train(cv::InputArray train_descriptors) = 0;
  virtual bool match(cv::InputArray query_descriptors, /*out*/ std::vector<cv::DMatch> & output_matches) = 0;
};


#endif /* __c_feature2d_matcher_h__ */
