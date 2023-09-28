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

struct c_sparse_feature_extractor_and_matcher_options
{
  // Detector options
  c_sparse_feature_detector_options detector;

  // Descriptor options
  c_sparse_feature_descriptor_options descriptor;

  // Matcher options
  c_feature2d_matcher_options matcher;
};


class c_sparse_feature_extractor_and_matcher
{
public:
  typedef c_sparse_feature_extractor_and_matcher this_class;
  typedef std::shared_ptr<this_class> sptr;

  static sptr create(const c_sparse_feature_extractor_and_matcher_options & options);

  const c_sparse_feature_extractor_and_matcher_options & options() const;
  const c_feature2d::sptr & detector() const;
  const c_feature2d::sptr & descriptor() const;
  const c_feature2d_matcher::sptr & matcher() const;

  SPARSE_FEATURE_DETECTOR_TYPE detector_type() const;
  SPARSE_FEATURE_DESCRIPTOR_TYPE descriptor_type() const;
  FEATURE2D_MATCHER_TYPE matcher_type() const;

  bool setup_reference_frame(cv::InputArray image,
      cv::InputArray mask = cv::noArray());

  bool match_current_frame(cv::InputArray image,
      cv::InputArray mask = cv::noArray());

  const cv::Mat & referece_image() const;
  const cv::Mat & referece_mask() const;
  const cv::Mat & referece_descriptors() const;
  const std::vector<cv::KeyPoint> & referece_keypoints() const;
  const std::vector<cv::Point2f> & matched_reference_positions() const;

  const cv::Mat & current_image() const;
  const cv::Mat & current_mask() const;
  const cv::Mat & current_descriptors() const;
  const std::vector<cv::KeyPoint> & current_keypoints() const;
  const std::vector<cv::Point2f> & matched_current_positions() const;

  void detect(cv::InputArray image, CV_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::InputArray mask = cv::noArray()) const;

  void detectAndCompute(cv::InputArray image, cv::InputArray mask,
      CV_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors,
      bool useProvidedKeypoints = false);

protected:
  c_sparse_feature_extractor_and_matcher();
  c_sparse_feature_extractor_and_matcher(const c_sparse_feature_extractor_and_matcher_options & opts);
  void extract_positions(const std::vector<cv::KeyPoint> & keypoints, std::vector<cv::Point2f> & positions) const;

  c_sparse_feature_extractor_and_matcher_options options_;
  c_feature2d::sptr detector_;
  c_feature2d::sptr descriptor_;
  c_feature2d_matcher::sptr matcher_;

  cv::Mat reference_image_;
  cv::Mat reference_mask_;
  cv::Mat reference_descriptors_;
  std::vector<cv::KeyPoint> reference_keypoints_;
  std::vector<cv::Point2f> reference_positions_;
  std::vector<cv::Point2f> matched_reference_positions_;

  cv::Mat current_image_;
  cv::Mat current_mask_;
  cv::Mat current_descriptors_;
  std::vector<cv::KeyPoint> current_keypoints_;
  std::vector<cv::Point2f> current_positions_;
  std::vector<cv::Point2f> matched_current_positions_;

  std::vector<cv::DMatch> current_matches_;
};

//struct c_feature2d_options
//{
//  double scale = 0.5;
//  c_sparse_feature_extractor_options sparse_feature_extractor;
//  c_feature2d_matcher_options sparse_feature_matcher;
//};


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
