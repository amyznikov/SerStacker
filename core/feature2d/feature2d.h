/*
 * feature2d.h
 *
 *  Created on: Jan 4, 2022
 *      Author: amyznikov
 *
 *  Sparse feature extraction and matching
 */

#pragma once
#ifndef __feature2d_h__
#define __feature2d_h__

#include "feature_extraction.h"  // Sparse feature extraction
#include "feature_matching.h"   // Sparse feature matching


/**
 * draw_matched_positions()
 *
 * Utility routine to draw sparse feature matches.
 *
 * Similar to cv::drawMatches()
 *
 */
bool draw_matched_positions(cv::OutputArray all_matches_image,
    cv::OutputArray selected_matches_image,
    cv::InputArray current_image,
    cv::InputArray reference_image,
    const std::vector<cv::Point2f> & matched_current_positions,
    const std::vector<cv::Point2f> & matched_reference_positions,
    cv::InputArray mask = cv::noArray());

/**
 * draw_matched_keyppints()
 *
 * Utility routine to draw sparse feature matches.
 *
 * Similar to cv::drawMatches()
 *
 */
bool draw_matched_keyppints(cv::OutputArray all_matches_image,
    cv::OutputArray selected_matches_image,
    cv::InputArray current_image,
    cv::InputArray reference_image,
    const std::vector<cv::KeyPoint> & current_keypoints,
    const std::vector<cv::KeyPoint> & reference_keypoints,
    const std::vector<cv::DMatch> & matches,
    cv::InputArray mask = cv::noArray());


/**
 * save_matched_points_in_octave_format()
 *
 * Utility routine to save sparse feature matches into text file in GNU octave format,
 *  as rectested by @iuspeniev .
 *
 * Dump matched keypoints (inliers only) into specified text file in octave format.
 * If image_size is not empty then all keypoimnts will shifted relative to image center as asked iuspeniev.
 */
bool save_matched_points_in_octave_format(const std::string & output_file_name,
    const std::vector<cv::Point2f> & matched_reference_keypoints,
    const std::vector<cv::Point2f> & matched_current_keypoints,
    const cv::InputArray inliers_mask,
    const cv::Size & image_size = cv::Size());

/**
 * save_matched_points_in_csv_format()
 *
 * Utility routine to dump sparse feature matches into text file in CSV format.
 *
 */
bool save_matches_in_csv_format(const std::string & output_file_name,
    const std::vector<cv::DMatch> & matches,
    const std::vector<cv::KeyPoint> & query_keypoints,
    const std::vector<cv::KeyPoint> & train_keypoints,
    cv::InputArray mask = cv::noArray());

/**
 * save_matched_positions_in_csv_format()
 *
 * Utility routine to dump matched positions into text file in CSV format.
 *
 */
bool save_matched_positions_in_csv_format(const std::string & output_file_name,
    const std::vector<cv::Point2f> & query_keypoints,
    const std::vector<cv::Point2f> & train_keypoints,
    cv::InputArray mask = cv::noArray());

#endif /* __feature2d_h__ */
