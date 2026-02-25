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


template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_sparse_feature_extractor_and_matcher_options> & ctx)
{
  using S = c_sparse_feature_extractor_and_matcher_options;

  ctlbind_expandable_group(ctls, "Feature2D detector", "");
    ctlbind(ctls, "", ctx(&S::detector), "");
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Feature2D descriptor", "");
    ctlbind(ctls, "", ctx(&S::descriptor), "");
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Feature2D matcher", "");
    ctlbind(ctls, "", ctx(&S::matcher), "");
  ctlbind_end_group(ctls);
}


class c_sparse_feature_extractor_and_matcher
{
public:
  typedef c_sparse_feature_extractor_and_matcher this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

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

//  const cv::Mat & referece_image() const;
//  const cv::Mat & referece_mask() const;
  const cv::Mat & referece_descriptors() const;
  const std::vector<cv::KeyPoint> & referece_keypoints() const;
  const std::vector<cv::Point2f> & matched_reference_positions() const;

//  const cv::Mat & current_image() const;
//  const cv::Mat & current_mask() const;
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

  c_sparse_feature_extractor_and_matcher_options _options;
  c_feature2d::sptr _detector;
  c_feature2d::sptr _descriptor;
  c_feature2d_matcher::sptr _matcher;

  cv::Mat _reference_image;
  cv::Mat _reference_mask;
  cv::Mat _reference_descriptors;
  std::vector<cv::KeyPoint> _reference_keypoints;
  std::vector<cv::Point2f> _reference_positions;
  std::vector<cv::Point2f> _matched_reference_positions;

  cv::Mat _current_descriptors;
  std::vector<cv::KeyPoint> _current_keypoints;
  std::vector<cv::Point2f> _current_positions;
  std::vector<cv::Point2f> _matched_current_positions;

  std::vector<cv::DMatch> _current_matches;
};

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
