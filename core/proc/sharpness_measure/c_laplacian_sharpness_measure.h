/*
 * c_laplacian_sharpness_measure.h
 *
 *  Created on: Sep 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_laplacian_sharpness_measure_h__
#define __c_laplacian_sharpness_measure_h__

#include "c_image_sharpness_measure.h"

class c_laplacian_sharpness_measure :
    public c_image_sharpness_measure
{
public:

  c_laplacian_sharpness_measure();
  c_laplacian_sharpness_measure(int dscale, const cv::Size & size);

  void set_dscale(int v);
  int dscale() const;

  void set_se_size(const cv::Size & v);
  const cv::Size & se_size() const;

  cv::Scalar compute(cv::InputArray image, cv::InputArray mask = cv::noArray()) const override;

  bool create_map(cv::InputArray image, cv::OutputArray output_map) const override;

  static bool compute(cv::InputArray image, cv::InputArray mask,
      int dscale, const cv::Size & se_size,
      cv::Scalar * output_sharpness_measure);

  static bool create_map(cv::InputArray image,
      int dscale, const cv::Size & se_size,
      cv::OutputArray output_map);

protected:
  int dscale_ = 2;
  cv::Size se_size_ = cv::Size(5, 5);
};

#endif /* __c_laplacian_sharpness_measure_h__ */
