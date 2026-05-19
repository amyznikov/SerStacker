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
#include <core/ctrlbind/ctrlbind.h>

/* Base options for sparse feature2d matchers */
struct c_feature2d_matcher_base_options
{
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_feature2d_matcher_base_options> & ctx)
{
  using S = c_feature2d_matcher_base_options;
}


/** @brief Sparse feature2d descriptor matcher abstract base class
 */
class c_feature2d_matcher
{
public:
  typedef c_feature2d_matcher this_class;
  typedef cv::Ptr<this_class> sptr;

  c_feature2d_matcher(int type = -1)  :
    _type(type)
  {
  }

  void set_type(int v)
  {
    _type = v;
  }

  int type() const
  {
    return _type;
  }

  void set_octavedif(int v)
  {
    _octavedif = v;
  }

  int octavedif() const
  {
    return _octavedif;
  }

  virtual ~c_feature2d_matcher() = default;
  virtual bool train(const std::vector<cv::KeyPoint> & train_keypoints, cv::InputArray train_descriptors) = 0;
  virtual bool match(const std::vector<cv::KeyPoint> & query_keypoints, cv::InputArray query_descriptors,
      /*out*/ std::vector<cv::DMatch> & output_matches) = 0;

protected:
  int _type = -1;
  int _octavedif = -1;
};


#endif /* __c_feature2d_matcher_h__ */
