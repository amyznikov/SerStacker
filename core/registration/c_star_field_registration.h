/*
 * c_star_field_registration.h
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#ifndef __c_star_field_registration__h__
#define __c_star_field_registration__h__

#include <core/proc/starmatch.h>
#include "c_frame_registration.h"

struct c_star_field_registration_options {
  // empty yet
};

class c_star_field_registration
    : public c_frame_registration
{
public:

  typedef c_star_field_registration this_class;
  typedef c_frame_registration base;
  typedef std::shared_ptr<this_class> ptr;

  static this_class::ptr create();
  static this_class::ptr create(const c_star_field_registration_options & opts);
  static this_class::ptr create(const c_frame_registration_base_options & base_opts, const c_star_field_registration_options & opts);

public: // ops
  bool setup_referece_frame(cv::InputArray image,
      cv::InputArray mask = cv::noArray()) override;

  c_star_field_registration_options & options();
  const c_star_field_registration_options & options() const ;

protected:
  bool create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk) const override;

  bool create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const override;

  bool extract_reference_features(cv::InputArray reference_feature_image,
      cv::InputArray reference_feature_mask) override;;

  bool estimate_feature_transform(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      cv::Mat1f * current_transform) override;




protected:
  c_star_field_registration();
  c_star_field_registration(const c_star_field_registration_options & opts);
  c_star_field_registration(const c_frame_registration_base_options & base_opts, const c_star_field_registration_options & opts);

  c_star_field_registration_options options_;

  std::vector<cv::KeyPoint> reference_keypoints_;
  std::vector<cv::Vec3w> reference_triangles_;
  std::vector<cv::Vec2f> reference_descriptors_;
  std::vector<cv::Point2f> reference_positions_;
  cv::Ptr<cvflann::KDTreeIndex<StarMathingDistanceType>> reference_index_;

  std::vector<cv::KeyPoint> current_keypoints_;
  std::vector<cv::Vec3w> current_triangles_;
  std::vector<cv::Vec2f> current_descriptors_;
  std::vector<cv::Point2f> current_positions_;

  std::vector<std::pair<uint16_t, uint16_t> > current_matches_;
};

#endif /* __c_star_field_registration__h__ */
