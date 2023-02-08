/*
 * c_gaussian_filter.h
 *
 *  Created on: Aug 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_gaussian_filter_h__
#define __c_gaussian_filter_h__

#include <opencv2/opencv.hpp>

/**
 * This is wrapper around of cv:sepFilter2D() with support of image masks.
 * Pixels for which mask == 0 don't contribute the blur operation.
 */

class c_gaussian_filter
{
public:
  c_gaussian_filter();

  c_gaussian_filter(double sigmaX, double sigmaY, const cv::Size & ksize = cv::Size(), double scale = 1.0);


  // The src pixels MUST be also set to 0 where _mask is 0
  void apply(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
      int borderType = cv::BORDER_DEFAULT, int ddepth=-1) const;

  double sigmax() const;
  double sigmay() const;

protected:
  static void create_gaussian_kernels(cv::Mat & kx, cv::Mat & ky, int ktype,
      cv::Size & ksize, double sigma1, double sigma2, double scale);

protected:
  double sigmaX_ = 0, sigmaY_ = 0, scale_ = 1.0;
  cv::Size ksize_;
  cv::Mat Kx_, Ky_;
};
#endif /* __c_gaussian_filter_h__ */
