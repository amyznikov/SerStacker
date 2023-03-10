/*
 * c_regular_stereo_matcher.h
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_regular_stereo_matcher_h__
#define __c_regular_stereo_matcher_h__

#include <opencv2/opencv.hpp>
#include <memory>

class c_regular_stereo_matcher
{
public:

  typedef c_regular_stereo_matcher this_clss;

  c_regular_stereo_matcher();

  bool match(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::InputArray referenceImage, cv::InputArray referenceMask,
      cv::Mat2f & outputMatches);

  void set_max_disparity(int v);
  int max_disparity() const;

  void set_max_scale(int v);
  int max_scale() const;

protected:
  int max_disparity_ = 128;
  int max_scale_ = 2;
};

#endif /* __c_regular_stereo_matcher_h__ */
