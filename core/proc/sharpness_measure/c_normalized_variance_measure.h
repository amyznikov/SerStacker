/*
 * c_normalized_variance_measure.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 *
 * Normalized image variance from paper
 *  "Autofocusing Algorithm Selection in Computer Microscopy"
 *    by Yu Sun and Stefan Duthaler and Bradley J. Nelson
 *
 *  measure = stdev(image) / mean(image)
 */

#pragma once
#ifndef __c_normalized_variance_measure_h__
#define __c_normalized_variance_measure_h__

#include "c_image_sharpness_measure.h"

class c_normalized_variance_measure :
    public c_image_sharpness_measure
{
public:
  typedef c_normalized_variance_measure this_class;
  typedef c_image_sharpness_measure base;

  void set_avgchannel(bool v);
  bool avgchannel() const;

  cv::Scalar compute(cv::InputArray image) const override;
  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static bool compute(cv::InputArray image, cv::OutputArray output_map, bool avgchannel,
      cv::Scalar * output_sharpness_metric);

protected:
  bool avgchannel_ = false;
};

#endif /* __c_normalized_variance_measure_h__ */
