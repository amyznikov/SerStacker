/*
 * c_image_sharpness_measure.h
 *
 *  Created on: Jan 24, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_sharpness_measure_h__
#define __c_image_sharpness_measure_h__

#include <opencv2/opencv.hpp>

class c_image_sharpness_measure
{
public:
  typedef c_image_sharpness_measure this_class;

  virtual ~c_image_sharpness_measure() = default;

  virtual cv::Scalar compute(cv::InputArray image) const = 0;
  virtual bool create_map(cv::InputArray image, cv::OutputArray output_map) const = 0;
};

#endif /* __c_image_sharpness_measure_h__ */
