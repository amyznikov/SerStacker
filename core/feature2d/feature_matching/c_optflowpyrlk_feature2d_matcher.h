/*
 * c_optflowpyrlk_feature2d_matcher.h
 *
 *  Created on: Sep 23, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_optflowpyrlk_feature2d_matcher_h__
#define __c_optflowpyrlk_feature2d_matcher_h__

#include "c_feature2d_matcher.h"


struct c_optflowpyrlk_feature2d_matcher_options:
    c_feature2d_matcher_base_options
{
  int maxLevel = 3;
  cv::Size winSize = cv::Size(21, 21);
  int maxIterations = 30;
  int flags = 0;
  double eps = 0.01;
  double minEigThreshold = 1e-4;
  double maxErr = 0;
};

bool match_optflowpyrlk(cv::InputArray previous_image, cv::InputArray next_image,
    const std::vector<cv::KeyPoint> & previous_keypoints,
    const c_optflowpyrlk_feature2d_matcher_options & opts,
    /* out */ std::vector<cv::Point2f> & matched_previous_positions,
    /* out */ std::vector<cv::Point2f> & matched_predicted_previous_positions);


#endif /* __c_optflowpyrlk_feature2d_matcher_h__ */
