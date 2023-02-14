/*
 * c_jovian_derotation.h
 *
 *  Created on: Sep 7, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __jupiter_h__
#define __jupiter_h__

#include <opencv2/opencv.hpp>
#include "image_transform.h"
#include "ecc_motion_model.h"
#include "c_jovian_ellipse_detector.h"

/**
 *
 */
class c_jovian_derotation
{
public:
  bool setup_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool compute(cv::InputArray input_image,
      cv::InputArray input_mask = cv::noArray());

  void set_jovian_detector_options(const c_jovian_ellipse_detector_options & v);
  const c_jovian_ellipse_detector_options & jovian_detector_options() const;

  // set mininum rotation angle for best rotation search range.
  void set_min_rotation(double v);
  double min_rotation() const;

  // set maximum rotation angle for best rotation search range.
  void set_max_rotation(double v);
  double max_rotation() const;

  void set_num_orientations(int v);
  int num_orientations() const;

  void set_eccflow_support_scale(int v);
  int eccflow_support_scale() const;

  void set_eccflow_normalization_scale(int v);
  int eccflow_normalization_scale() const;

  void set_eccflow_max_pyramid_level(int v);
  int max_eccflow_pyramid_level(int v);

  void set_force_reference_ellipse(bool v);
  bool force_reference_ellipse() const;

  const cv::RotatedRect & reference_ellipse() const;
  const cv::RotatedRect & current_ellipse() const;

  const cv::Mat1b & reference_ellipse_mask() const ;
  const cv::Mat1b & current_ellipse_mask() const ;

  const cv::Mat2f & current_derotation_remap() const;
  const cv::Mat1f & current_wmask() const;

  void set_debug_path(const std::string & v);
  const std::string & debug_path() const;

protected:
  static double compute_jovian_derotation_cost(const cv::Mat1f & reference_component_image,
      const cv::Mat1f & curret_rotated_component_image,
      const cv::Mat1f & rotation_mask,
      cv::Mat1f * difference_image = nullptr) ;

protected:

  cv::RotatedRect reference_ellipse_;
  cv::Mat1f reference_gray_image_;
  cv::Mat1f reference_normalized_image_;
  cv::Mat1b reference_ellipse_mask_;

  cv::RotatedRect current_ellipse_;
  cv::Mat1f current_gray_image_;
  cv::Mat1f current_normalized_image_;
  cv::Mat1b current_ellipse_mask_;


  cv::Mat1f current_wmask_;
  cv::Mat2f current_remap_;

  ecc2::c_eccflow eccflow_;
  c_jovian_ellipse_detector planetary_detector_;

  double min_rotation_ = -30 * CV_PI / 180;
  double max_rotation_ = +30 * CV_PI / 180;
  int num_orientations_ = 1;
  int eccflow_support_scale_ = 4;
  int eccflow_normalization_scale_ = 0;
  int eccflow_max_pyramid_level_ = 0;
  bool force_reference_ellipse_ = false;

  std::string debug_path_;
};




/**
 *
  cv::Mat input_image, reference_image;
  cv::RotatedRect ellipse;
  cv::Mat2f rmap;
  cv::Mat1f wmask;
 ...

  c_ecc_forward_additive ecc(ECC_MOTION_EUCLIDEAN_SCALED);
  c_ecc_pyramide_align ecch(&ecc);

  cv::Matx23d T =
    createEyeTransform(ecc.motion_type();

  if ( !ecch.align(input_image, reference_image, T)) ) {
    CF_ERROR("ecch.align() fails");
    return false;
  }

  if ( !fit_jovian_ellipse(reference_image, &ellipse) ) {
    CF_ERROR("fit_jovian_ellipse() fails");
    return false;
  }

  create_jovian_rotation_remap(-8.5 * CV_PI / 180,
    ellipse,
    T,
    reference_image.size(),
    rmap,
    wmask);

  cv::remap(input_image, input_image, rmap, cv::noArray(),
      cv::INTER_LINEAR);


  FIXME: generalize ref2input_transform to allow arbitrary
        remaps between reference and input images
 */
void create_jovian_rotation_remap(double longitude_rotation_angle,
    const cv::RotatedRect & ellipse,
    const cv::Matx23d & ref2input_affine_transform,
    const cv::Size & remap_size,
    cv::Mat2f & output_remap,
    cv::Mat1f & output_wmask);


#endif /* __jupiter_h__ */
