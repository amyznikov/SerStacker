/*
 * jupiter.h
 *
 *  Created on: Sep 7, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __jupiter_h__
#define __jupiter_h__

#include <opencv2/opencv.hpp>
#include <core/proc/image_registration/image_transform.h>
#include <core/proc/image_registration/ecc_motion_model.h>



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
void create_jovian_rotation_remap(double rotation_angle,
    const cv::RotatedRect & ellipse,
    const cv::Matx23d & ref2input_affine_transform,
    const cv::Size & remap_size,
    cv::Mat2f & output_remap,
    cv::Mat1f & output_wmask);

struct c_jovian_ellipse_detector_options {
  std::vector<float> hlines;
  double stdev_factor = 0.5;
  double normalization_blur = 2;
  double gradient_blur = 2;
  int normalization_scale = 3;
  bool force_reference_ellipse = false;
};

class c_jovian_ellipse_detector
{
public:
  void set_hlines(const std::vector<float> & v);
  const std::vector<float> & hlines() const;

  void set_normalization_scale(int v);
  int normalization_scale() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  void set_normalization_blur(double v);
  double normalization_blur() const;

  void set_gradient_blur(double v);
  double gradient_blur() const;

  void set_options(const c_jovian_ellipse_detector_options & v);
  const c_jovian_ellipse_detector_options & options() const;
  c_jovian_ellipse_detector_options & options();

  bool detect_planetary_disk(cv::InputArray _image, cv::InputArray mask = cv::noArray());

  const cv::Mat & detected_planetary_disk_mask() const;
  const cv::Mat & detected_planetary_disk_edge() const;
  const cv::RotatedRect & ellipseAMS() const;
  const cv::Mat & initial_artifial_ellipse_edge() const;
  const cv::Mat & remapped_artifial_ellipse_edge() const;
  const cv::Mat & aligned_artifial_ellipse_edge() const;
  const cv::Mat1b & aligned_artifial_ellipse_edge_mask() const;
  const cv::Mat1b & aligned_artifial_ellipse_mask() const;
  const cv::RotatedRect & ellipseAMS2() const;
  const cv::RotatedRect & planetary_disk_ellipse() const;

  // bounding box for component_mask
//  const cv::Rect & crop_bounding_box() const;
//
  const cv::Mat & gray_image() const ;
  const cv::Mat & normalized_image() const ;
//  const cv::Mat & cropped_gradient_image() const ;
//
//  const cv::Mat & initial_artificial_ellipse() const ;
//  const cv::Mat & initial_artificial_ellipse_fit() const ;
//  const cv::Mat & cropped_final_ellipse_fit() const ;

protected:
  c_jovian_ellipse_detector_options options_;

  cv::RotatedRect ellipse_;
  cv::Mat detected_planetary_disk_mask_;
  cv::Mat detected_planetary_disk_edge_;
  cv::RotatedRect ellipseAMS_;

  cv::Mat1f initial_artifial_ellipse_edge_;
  cv::Mat1f remapped_artifial_ellipse_edge_;
  cv::Mat1f aligned_artifial_ellipse_edge_;
  cv::Mat1b aligned_artifial_ellipse_edge_mask_;
  cv::Mat1b aligned_artifial_ellipse_mask_;
  cv::RotatedRect ellipseAMS2_;

  cv::Mat gray_image_;
  cv::Mat normalized_image_;

//  cv::Rect crop_bounding_box_;
//  cv::Mat cropped_gradient_image_;
//  cv::Mat cropped_artificial_ellipse_;
//  cv::Mat cropped_initial_ellipse_fit_;
//  cv::Mat cropped_final_ellipse_fit_;
};


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

//  void set_hlines(const std::vector<float> & hlines);
//  const std::vector<float> & hlines() const;

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




#endif /* __jupiter_h__ */
