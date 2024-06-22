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
    public c_ecc_motion_model_template<c_euclidean_image_transform>
{
public:
  typedef c_euclidean_ecc_motion_model this_class;
  typedef c_ecc_motion_model_template<c_euclidean_image_transform> base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_euclidean_ecc_motion_model(c_euclidean_image_transform * transform = nullptr) :
      base(transform)
  {
  }

public: // c_ecc_motion_model

  int num_adustable_parameters() const final;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const final;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) final;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) final;
};

#endif /* __c_euclidean_ecc_motion_model_h__ */
