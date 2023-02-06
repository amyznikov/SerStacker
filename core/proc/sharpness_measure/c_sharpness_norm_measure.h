/*
 * c_sharpness_norm_measure.h
 *
 *  Created on: Feb 7, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sharpness_norm_measure_h__
#define __c_sharpness_norm_measure_h__

#include <opencv2/opencv.hpp>

class c_sharpness_norm_measure
{
public:
  typedef c_sharpness_norm_measure this_class;
  typedef std::shared_ptr<this_class> ptr;

  void set_norm_type(cv::NormTypes v);
  cv::NormTypes norm_type() const;

  void set_sigma(double v);
  double sigma() const;

  double measure(cv::InputArray src, cv::InputArray mask = cv::noArray()) const;
  static double measure(cv::InputArray src, cv::InputArray mask, double sigma, cv::NormTypes norm_type);

  double add(cv::InputArray src, cv::InputArray mask = cv::noArray());
  double average() const;
  void reset();

  double accumulator() const;
  int counter() const;

protected:
  double sigma_ = 1; // M_SQRT2;
  double accumulator_ = 0;
  int counter_ = 0;
  cv::NormTypes norm_type_ = cv::NORM_L1;

};

#endif /* __c_sharpness_norm_measure_h__ */
