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
  //cv::Size crop_size;
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
  static this_class::ptr create(const c_frame_registration_base_options & base_opts, const c_planetary_disk_registration_options & opts);

public: // parameters

  const cv::Point2f & current_centroid() const;
  const cv::Point2f & reference_centroid() const;

  c_planetary_disk_registration_options & options();
  const c_planetary_disk_registration_options & options() const ;

protected: // overrides
  bool create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk) const override;

  bool create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const override;

  bool extract_reference_features(cv::InputArray reference_feature_image,
      cv::InputArray reference_feature_mask) override;

  bool estimate_feature_transform(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      cv::Mat1f * current_transform) override;

//  bool select_crop_rectangle(const cv::Size & feature_image_size,
//      const cv::Size & crop_size,
//      const cv::Point2f & crop_center,
//      cv::Rect * ROI) const;

protected:
  c_planetary_disk_registration();
  c_planetary_disk_registration(const c_planetary_disk_registration_options & opts);
  c_planetary_disk_registration(const c_frame_registration_base_options & base_opts, const c_planetary_disk_registration_options & opts);

protected:
  c_planetary_disk_registration_options options_;

  cv::Point2f reference_centroid_ = cv::Point2f(-1, -1);
  cv::Point2f current_centroid_ = cv::Point2f(-1, -1);
};



#endif /* __c_planetary_disk_image_registration_h__ */
