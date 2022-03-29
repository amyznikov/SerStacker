/*
 * c_feature_based_image_registration.h
 *
 *  Created on: Sep 18, 2019
 *      Author: amyznikov
 */

#ifndef __c_feature_based_image_registration_h___
#define __c_feature_based_image_registration_h___

//#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <core/feature2d/feature2d.h>
#include "c_frame_registration.h"

struct c_feature_based_registration_options {
  c_sparse_feature_extractor_options sparse_feature_extractor;
  c_feature2d_matcher_options sparse_feature_matcher;
};

class c_feature_based_registration
    : public c_frame_registration
{
public:
  typedef c_feature_based_registration this_class;
  typedef c_frame_registration base;

  typedef std::shared_ptr<this_class> ptr;

  static this_class::ptr create();
  static this_class::ptr create(const c_feature_based_registration_options & feature_options);
  static this_class::ptr create(const c_frame_registration_base_options & base_opts,
      const c_feature_based_registration_options & feature_opts);

public: // parameters

  const c_feature2d::ptr & set_keypoints_detector(const c_feature2d::ptr & detector);
  const c_feature2d::ptr & keypoints_detector() const;

  // for cv::xfeatures2d::SURF::create()
  //void set_feature_hessian_threshold(double v);
  //double feature_hessian_threshold() const;

  const std::vector<cv::KeyPoint> & reference_keypoints() const;
  const cv::Mat & reference_descriptors() const;
  const std::vector<cv::KeyPoint> & current_keypoints() const;
  const cv::Mat & current_descriptors() const;

  c_feature_based_registration_options & feature_options();
  const c_feature_based_registration_options & feature_options() const;

public: // overrides, made public for debug & non-regular usage
  virtual c_feature2d::ptr create_keypoints_detector() const;

  bool create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk) const override;

  bool create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const override;

  bool extract_reference_features(cv::InputArray reference_feature_image,
      cv::InputArray reference_feature_mask) override;

  bool detect_and_match_keypoints(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      std::vector<cv::Point2f> & output_matched_current_positions,
      std::vector<cv::Point2f> & output_matched_reference_positions,
      std::vector<cv::KeyPoint> * current_keypoints = nullptr,
      cv::Mat * current_descriptors = nullptr,
      std::vector<cv::DMatch> * current_matches = nullptr,
      std::vector<std::vector<cv::DMatch> > * current_matches12 = nullptr ) const;

  bool estimate_feature_transform(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      cv::Mat1f * current_transform) override;


protected: // use create() instead
  c_feature_based_registration();
  c_feature_based_registration(const c_feature_based_registration_options & opts);
  c_feature_based_registration(const c_frame_registration_base_options & base_opts, const c_feature_based_registration_options & opts);

protected:
  c_feature_based_registration_options feature_options_;

  c_feature2d::ptr keypoints_detector_;
  //cv::Ptr<cv::FlannBasedMatcher> keypoints_matcher_;
  c_feature2d_matcher::ptr keypoints_matcher_;

  std::vector<cv::KeyPoint> reference_keypoints_;
  cv::Mat reference_descriptors_;

  std::vector<cv::KeyPoint> current_keypoints_;
  cv::Mat current_descriptors_;

  std::vector<cv::DMatch> current_matches_;
  std::vector<std::vector<cv::DMatch> > current_matches12_;

  std::vector<cv::Point2f> matched_current_positions_;
  std::vector<cv::Point2f> matched_reference_positions_;
};



#endif /* __c_feature_based_image_registration_h___ */
