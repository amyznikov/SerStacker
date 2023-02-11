/*
 * c_affine_ecc_motion_model.h
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_affine_ecc_motion_model_h__
#define __c_affine_ecc_motion_model_h__

#include "c_image_transform.h"
#include "ecc2.h"

class c_affine_ecc_motion_model:
    public c_affine_image_transform,
    public ecc2::c_ecc_motion_model
{
public:
  typedef c_affine_ecc_motion_model this_class;
  typedef c_affine_image_transform transform;
  typedef c_ecc_motion_model motion_model;

  c_affine_ecc_motion_model()
  {
  }

  c_affine_ecc_motion_model(const float a[2][3]) :
      transform(a)
  {
  }

  c_affine_ecc_motion_model(const cv::Matx23f & a) :
      transform(a)
  {
  }

  c_affine_ecc_motion_model(const cv::Mat1f & a) :
      transform(a)
  {
  }

  c_affine_ecc_motion_model(float a00, float a01, float a02, float a10, float a11, float a12) :
      transform(a00, a01, a02, a10, a11, a12)
  {
  }

  cv::Mat1f parameters() const override
  {
    return transform::parameters();
  }

  bool set_parameters(const cv::Mat1f & p) override
  {
    return transform::set_parameters(p);
  }

  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override
  {
    return transform::scale_transfrom(p, factor);
  }

  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override
  {
    return transform::create_remap(map, size);
  }

public: // c_ecc_motion_model
  int num_adustable_parameters() const override;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const override;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) override;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) override;

protected:
};

#endif /* __c_affine_ecc_motion_model_h__ */
