/*
 * c_average_with_mask.h
 *
 *  Created on: Sep 14, 2019
 *      Author: amyznikov
 */
#pragma once
#ifndef __c_average_with_mask_h__
#define __c_average_with_mask_h__

#include <opencv2/opencv.hpp>

class c_average_with_mask
{
  cv::Mat accumulator_, counter_;
  int nbframes_ = 0;
  static constexpr int acctype = CV_32F;

public:
  bool add(cv::InputArray src, cv::InputArray mask = cv::noArray());

  const cv::Mat & accumulator() const;
  const cv::Mat & counter() const;
  int nbframes() const;

  bool average(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double scale = 1.0, int dtype = acctype) const;

  void release();
};


#endif /* __c_average_with_mask_h__ */
