/*
 * c_feature_based_image_registration.h
 *
 *  Created on: Sep 18, 2019
 *      Author: amyznikov
 */

#ifndef __c_feature_based_image_registration_h___
#define __c_feature_based_image_registration_h___

#include "c_frame_registration.h"
#include <core/registration/c_sparse_feature_options.h>

class c_generic_frame_registration :
    public c_frame_registration
{
public:
  typedef c_generic_frame_registration this_class;
  typedef c_frame_registration base;

  typedef std::shared_ptr<this_class> ptr;

  static this_class::ptr create();
  static this_class::ptr create(const c_sparse_feature_options & feature_options);
  static this_class::ptr create(const c_frame_registration_base_options & base_opts,
      const c_sparse_feature_options & feature_opts);

public: // parameters

  const c_sparse_feature_extractor::ptr & set_keypoints_extractor(const c_sparse_feature_extractor::ptr & extractor);
  const c_sparse_feature_extractor::ptr & keypoints_extractor() const;

  const std::vector<cv::KeyPoint> & reference_keypoints() const;
  const cv::Mat & reference_descriptors() const;
  const std::vector<cv::KeyPoint> & current_keypoints() const;
  const cv::Mat & current_descriptors() const;

  c_sparse_feature_options & feature_options();
  const c_sparse_feature_options & feature_options() const;

public: // overrides, made public for debug & non-regular usage
  virtual c_sparse_feature_extractor::ptr create_keypoints_extractor() const;

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
  c_generic_frame_registration();
  c_generic_frame_registration(const c_sparse_feature_options & opts);
  c_generic_frame_registration(const c_frame_registration_base_options & base_opts,
      const c_sparse_feature_options & opts);

protected:
  c_sparse_feature_options feature_options_;

  c_sparse_feature_extractor::ptr keypoints_extractor_;
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
