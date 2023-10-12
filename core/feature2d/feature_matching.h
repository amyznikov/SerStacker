/*
 * feature_matching.h
 *
 *  Created on: Dec 26, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __feature_matching_h__
#define __feature_matching_h__

#include <core/feature2d/feature_matching/c_flann_based_feature2d_matcher.h>
#include <core/feature2d/feature_matching/c_hamming_distance_feature2d_matcher.h>
#include <core/feature2d/feature_matching/c_optflowpyrlk_feature2d_matcher.h>
#include <core/feature2d/feature_matching/c_triangle_matcher.h>
#include <core/feature2d/feature_matching/c_snorm_based_feature2d_matcher.h>

#include <core/ssprintf.h>

enum FEATURE2D_MATCHER_TYPE {
  FEATURE2D_MATCHER_UNKNOWN = -1,
  FEATURE2D_MATCHER_AUTO_SELECT = FEATURE2D_MATCHER_UNKNOWN,
  FEATURE2D_MATCHER_HAMMING,
  FEATURE2D_MATCHER_FLANN,
  FEATURE2D_MATCHER_SNORM,
  FEATURE2D_MATCHER_TRIANGLES,
  FEATURE2D_MATCHER_OptFlowPyrLK,
};

template<> const c_enum_member *
  members_of<FEATURE2D_MATCHER_TYPE>();


struct c_feature2d_matcher_options {

  FEATURE2D_MATCHER_TYPE type = FEATURE2D_MATCHER_AUTO_SELECT;

  c_hamming_distance_feature2d_matcher_options hamming;
  c_flann_based_feature2d_matcher_options flann;
  c_triangle_matcher_options triangles;
  c_optflowpyrlk_feature2d_matcher_options optflowpyrlk;
  c_snorm_based_feature2d_matcher_options snorm;

};

c_feature2d_matcher::sptr create_sparse_feature_matcher(
    const c_feature2d_matcher_options & options);



void dump_supported_feature2d_matchers(FILE * fp = stdout);





/** @brief
 *  match_keypoints()
 *
 *  This routine uses given keypoint matcher to match current and reference keypoints.
 */
size_t match_keypoints(const cv::Ptr<c_feature2d_matcher> & keypoints_matcher,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const cv::Mat & current_descriptors,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const cv::Mat & reference_descriptors,
    std::vector<cv::DMatch> * output_best_matches  = nullptr,
    std::vector<cv::Point2f> * output_matched_current_positions = nullptr,
    std::vector<cv::Point2f> * output_matched_reference_positions = nullptr);





#endif /* __feature_matching_h__ */
