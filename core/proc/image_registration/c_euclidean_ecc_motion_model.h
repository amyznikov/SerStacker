/*
 * c_euclidean_ecc_motion_model.h
 *
 *  Created on: Feb 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_euclidean_ecc_motion_model_h__
#define __c_euclidean_ecc_motion_model_h__

#include "c_image_transform.h"
#include "ecc2.h"


class c_euclidean_ecc_motion_model :
    public c_ecc_motion_model
{
public:
  typedef c_euclidean_ecc_motion_model this_class;
  typedef c_ecc_motion_model base;

  c_euclidean_ecc_motion_model(c_euclidean_image_transform * transform = nullptr) :
      transform_(transform)
  {
  }

  void set_transform(c_euclidean_image_transform * transform)
  {
    transform_ = transform;
  }

  c_euclidean_image_transform * transform() const
  {
    return transform_;
  }

  cv::Mat1f parameters() const override
  {
    return transform_ ? transform_->parameters() : cv::Mat1f();
  }

  bool set_parameters(const cv::Mat1f & p) override
  {
    return transform_ ? transform_->set_parameters(p): false;
  }

  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override
  {
    return transform_ ? transform_->scale_transfrom(p, factor) : cv::Mat1f();
  }

  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override
  {
    return transform_ ? transform_-> create_remap(map, size) : false;
  }

public: // c_ecc_motion_model

  void set_fix_translation(bool v)
  {
    fix_translation_ = v;
  }

  bool fix_translation() const
  {
    return fix_translation_;
  }

  void set_fix_rotation(bool v)
  {
    fix_rotation_ = v;
  }

  bool fix_rotation() const
  {
    return fix_rotation_;
  }

  void set_fix_scale(bool v)
  {
    fix_scale_ = v;
  }

  bool fix_scale() const
  {
    return fix_scale_;
  }

  int num_adustable_parameters() const override;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const override;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) override;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) override;

protected:
  c_euclidean_image_transform * transform_ = nullptr;
  bool fix_translation_ = false;
  bool fix_rotation_ = false;
  bool fix_scale_ = false;
};

#endif /* __c_euclidean_ecc_motion_model_h__ */
