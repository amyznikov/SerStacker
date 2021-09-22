/*
 * c_planetary_disk_image_registration.h
 *
 *  Created on: Aug 28, 2020
 *      Author: amyznikov
 */

#ifndef __c_planetary_disk_image_registration_h__
#define __c_planetary_disk_image_registration_h__

#include "c_frame_registration.h"

struct c_planetary_disk_registration_options {
  bool align_planetary_disk_masks = false;
};

class c_planetary_disk_registration
    : public c_frame_registration
{
public:
  typedef c_planetary_disk_registration this_class;
  typedef c_frame_registration base;

  typedef std::shared_ptr<this_class> ptr;

  static this_class::ptr create();
  static this_class::ptr create(const c_planetary_disk_registration_options & opts);
  static this_class::ptr create(const c_frame_registration_base_options & base_opts,
      const c_planetary_disk_registration_options & opts);

public: // parameters

  const cv::Point2f & current_centroid() const;
  const cv::Point2f & reference_centroid() const;

  const c_planetary_disk_registration_options & planetary_disk_registration_options() const ;
  c_planetary_disk_registration_options & planetary_disk_registration_options();

protected: // overrides
  bool create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk) const override;

  bool extract_reference_features(cv::InputArray reference_feature_image,
      cv::InputArray reference_feature_mask) override;

  bool estimate_feature_transform(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      cv::Mat1f * current_transform) override;

  bool create_reference_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const override;

  bool create_current_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const override;

protected:
  c_planetary_disk_registration();
  c_planetary_disk_registration(const c_planetary_disk_registration_options & opts);
  c_planetary_disk_registration(const c_frame_registration_base_options & base_opts,
      const c_planetary_disk_registration_options & opts);

protected:
  c_planetary_disk_registration_options options_;

  cv::Point2f reference_centroid_ = cv::Point2f(-1, -1);
  cv::Point2f current_centroid_ = cv::Point2f(-1, -1);

  cv::Rect current_component_rect_;
  cv::Rect reference_component_rect_;
  cv::Mat current_component_mask_;
  cv::Mat reference_component_mask_;
};



#endif /* __c_planetary_disk_image_registration_h__ */
