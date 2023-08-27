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
  bool setup_jovian_ellipse(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool setup_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool compute(cv::InputArray input_image,
      cv::InputArray input_mask = cv::noArray());

  void set_jovian_detector_options(const c_jovian_ellipse_detector_options & v);
  const c_jovian_ellipse_detector_options & jovian_detector_options() const;

  void set_max_pyramid_level(int v);
  int max_pyramid_level() const;

  // set mininum rotation angle for best rotation search range.
  void set_min_rotation(double v);
  double min_rotation() const;

  // set maximum rotation angle for best rotation search range.
  void set_max_rotation(double v);
  double max_rotation() const;

  void set_num_orientations(int v);
  int num_orientations() const;

  const cv::RotatedRect & jovian_ellipse() const;
  const cv::Mat1b & jovian_ellipse_mask() const ;

  const cv::Mat2f & current_derotation_remap() const;
  const cv::Mat1f & current_wmask() const;

  void set_debug_path(const std::string & v);
  const std::string & debug_path() const;

protected:
  static double compute_jovian_derotation_cost(const cv::Mat1f & reference_component_image,
      const cv::Mat1f & curret_rotated_component_image,
      const cv::Mat1f & rotation_mask,
      cv::Mat1f * difference_image = nullptr) ;

  static int estimate_max_pyramid_level(const cv::RotatedRect & ellipse,
      int pyramid_minimum_ellipse_size);

protected:

  cv::RotatedRect jovian_ellipse_;
  cv::Rect jovian_crop_;
  cv::Mat1b jovian_ellipse_mask_;

  cv::Mat1f current_image_;
  cv::Mat1f reference_image_;
  cv::Mat1b current_mask_;
  cv::Mat1b reference_mask_;


  cv::Mat1f current_wmask_;
  cv::Mat2f current_remap_;

  c_eccflow eccflow_;
  c_jovian_ellipse_detector jovian_detector_;

  double min_rotation_ = -30 * CV_PI / 180;
  double max_rotation_ = +30 * CV_PI / 180;
  int max_pyramid_level_ = -1;
  int num_orientations_ = 3;

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
