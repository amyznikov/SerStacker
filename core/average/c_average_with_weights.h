/*
 * c_average_with_weights.h
 *
 *  Created on: Sep 14, 2019
 *      Author: amyznikov
 */
#pragma once
#ifndef __c_average_with_weights_h__
#define __c_average_with_weights_h__

#include <opencv2/opencv.hpp>

class c_average_with_weights
{
  cv::Mat accumulator_, weights_, tmp_;
  int nbframes_ = 0;

public:

  bool add(cv::InputArray src,
      cv::InputArray weights);

  bool average(cv::OutputArray avg,
      cv::OutputArray mask = cv::noArray(),
      double scale = 1.0,
      int ddepth = -1) const;

  void reset();

  const cv::Mat & accumulator() const {
    return accumulator_;
  }

  const cv::Mat & weights() const {
    return weights_;
  }

  int nbframes() const {
    return nbframes_;
  }

};

#endif /* __c_average_with_weights_h__ */
