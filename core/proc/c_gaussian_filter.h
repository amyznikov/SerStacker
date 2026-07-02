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

  double sigmax() const
  {
    return _sigmaX;
  }

  double sigmay() const
  {
    return _sigmaY;
  }

  const cv::Size & ksize() const
  {
    return _ksize;
  }

protected:
  static void create_gaussian_kernels(cv::Mat & kx, cv::Mat & ky, int ktype,
      cv::Size & ksize, double sigma1, double sigma2, double scale);

protected:
  double _sigmaX = 0, _sigmaY = 0;
  double _scale = 1.0;
  cv::Size _ksize;
  cv::Mat _Kx, _Ky;
};

void gaussian_filter(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
    const cv::Size2f & sigma, const cv::Size & ksize,
    double scale = 1, double delta = 0,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT,
    const cv::Scalar & borderValue = cv::Scalar());

void gaussian_hpass_filter(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst,
    const cv::Size2f & sigma, const cv::Size & ksize,
    double scale = 1, double delta = 0,
    cv::BorderTypes borderType = cv::BORDER_DEFAULT,
    const cv::Scalar & borderValue = cv::Scalar());


#endif /* __c_gaussian_filter_h__ */
