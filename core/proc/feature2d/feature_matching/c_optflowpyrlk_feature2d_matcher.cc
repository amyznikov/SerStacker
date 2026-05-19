/*
 * c_optflowpyrlk_feature2d_matcher.cc
 *
 *  Created on: Sep 23, 2023
 *      Author: amyznikov
 */

#include "c_optflowpyrlk_feature2d_matcher.h"
#include <core/debug.h>


bool match_optflowpyrlk(cv::InputArray previous_image, cv::InputArray next_image,
    const std::vector<cv::KeyPoint> & previous_keypoints,
    const c_optflowpyrlk_feature2d_matcher_options & opts,
    /* out */ std::vector<cv::Point2f> & matched_previous_positions,
    /* out */ std::vector<cv::Point2f> & matched_predicted_previous_positions)
{
  std::vector<cv::Point2f> previous_positions;
  std::vector<cv::Point2f> predicted_previous_positions;
  std::vector<uint8_t> status;
  std::vector<float> err;

  matched_previous_positions.clear();
  matched_predicted_previous_positions.clear();

  previous_positions.reserve(previous_keypoints.size());
  for( const cv::KeyPoint &p : previous_keypoints ) {
    previous_positions.emplace_back(p.pt);
  }

  cv::calcOpticalFlowPyrLK(previous_image, next_image,
      previous_positions, predicted_previous_positions,
      status, err,
      opts.winSize,
      opts.maxLevel,
      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, opts.maxIterations, opts.eps),
      opts.flags,
      opts.minEigThreshold);

  for( int i = 0, n = previous_positions.size(); i < n; ++i ) {
    if( status[i] && (opts.maxErr <= 0 || err[i] < opts.maxErr) ) {

      matched_previous_positions.emplace_back(previous_positions[i]);
      matched_predicted_previous_positions.emplace_back(predicted_previous_positions[i]);
    }
  }

  return true;
}

