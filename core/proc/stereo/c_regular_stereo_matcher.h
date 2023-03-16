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

  void set_kernel_sigma(double v);
  double kernel_sigma() const;

  void set_kernel_radius(int v);
  int kernel_radius() const;

  void set_debug_directory(const std::string & v);
  const std::string & debug_directory() const;

  const std::vector<cv::Point>& debug_points() const;
  void set_debug_points(const std::vector<cv::Point> & v);

protected:
  template<class MT>
  bool match_impl(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::InputArray referenceImage, cv::InputArray referenceMask,
      cv::Mat2f & outputMatches);

protected:
  int max_disparity_ = 128;
  int max_scale_ = 2;
  double kernel_sigma_ = 1;
  int kernel_radius_ = 3;

  std::string debug_directory_;
  std::vector<cv::Point> debug_points_;
};

#endif /* __c_regular_stereo_matcher_h__ */
