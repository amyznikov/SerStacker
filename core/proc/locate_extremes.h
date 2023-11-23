/*
 * locate_extremes.h
 *
 *  Created on: Nov 21, 2023
 *      Author: amyznikov
 *
 * locate local extremal points which are local maximums or local minimums
 * among theirs neighborhood defined by given Structuring Element SE
 *
 */

#pragma once
#ifndef __locate_extremes_h__
#define __locate_extremes_h__

#include <opencv2/opencv.hpp>


struct c_locate_extremes_options
{
  enum neighbor_filter_type
  {
    filter_morph,
    filter_mean,
  };

  neighbor_filter_type filter_type =
      filter_morph;

  cv::MorphShapes se_shape = cv::MORPH_RECT;
  cv::BorderTypes border_type = cv::BORDER_REPLICATE;
  cv::Scalar border_value;

  cv::Size se_size = cv::Size(3, 3);
  cv::Point anchor = cv::Point(-1, -1);


  bool locate_maximums = true;
  double maximums_alpha = 1;
  double maximums_beta = 0;

  bool locate_minimums = false;
  double minimums_alpha = 1;
  double minimums_beta = 0;
};

bool locate_extremes(cv::InputArray image, cv::InputArray input_mask,
    cv::OutputArray output_mask,
    const c_locate_extremes_options & opts);


#endif /* __locate_extremes_h__ */
