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
#include "eccalign.h"

/*
 Detect planetary disk on given image, fit Jovian ellipse
 and return final enclosing ellipse as cv::RotatedRect structure.

 Example:

  @code
  cv::RotatedRect rc;

  if ( !detect_jovian_ellipse(image, &rc) ) {
    CF_ERROR("fit_jovian_ellipse() fails");
    return;
  }

  cv::ellipse(image, rc, CV_RGB(0, 1, 0), 1, cv::LINE_8);

  // draw major semi-axis
  cv::line(image, rc.center, rc.center +
      0.5 * cv::Point2f(rc.size.width * cos(rc.angle * CV_PI / 180),
          rc.size.width * sin(rc.angle * CV_PI / 180)),
          CV_RGB(1,1,0));

  // draw minor semi-axis
  cv::line(image, rc.center, rc.center +
      0.5 * cv::Point2f(rc.size.height * sin(rc.angle * CV_PI / 180),
          -rc.size.height * cos(rc.angle * CV_PI / 180)),
          CV_RGB(1,1,0));

  save_image(image, "fit_jovian_ellipse.tiff");

 @endcode
 */
bool detect_jovian_ellipse(cv::InputArray _image,
    cv::RotatedRect * output_ellipse_rect,
    const std::string & debug_path = "");



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


/**
 *
 */
void get_jovian_ellipse_bounding_box(const cv::Size & image_size,
    const cv::RotatedRect & jovian_ellipse,
    cv::Rect * bbox);


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

  // set mininum rotation angle for best rotation search range.
  void set_min_rotation(double v);
  double min_rotation() const;

  // set maximum rotation angle for best rotation search range.
  void set_max_rotation(double v);
  double max_rotation() const;

  void set_eccflow_support_scale(int v);
  int eccflow_support_scale() const;

  void set_eccflow_normalization_scale(int v);
  int eccflow_normalization_scale() const;

  void set_eccflow_max_pyramid_level(int v);
  int max_eccflow_pyramid_level(int v);

  //  void set_align_jovian_disk_horizontally(bool v);
  //  bool align_jovian_disk_horizontally() const;

  const cv::RotatedRect & reference_ellipse() const;
  const cv::RotatedRect & current_ellipse() const;

  const cv::Rect & reference_boundig_box() const;
  const cv::Rect & current_boundig_box() const;

  const cv::Mat1b & reference_ellipse_mask() const;
  const cv::Mat1b & current_ellipse_mask() const;

  const cv::Mat2f & current_total_remap() const;
  const cv::Mat1f & current_total_mask() const;
  const cv::Mat1b & current_total_binary_mask() const;

  void set_enable_debug(bool v);
  bool enable_debug() const;

  void set_debug_path(const std::string & v);
  const std::string & debug_path() const;

protected:
  bool extract_jovian_image(cv::InputArray src_image, cv::InputArray src_mask,
      cv::RotatedRect * output_ellipse,
      cv::Rect * output_ellipse_boundig_box,
      cv::Mat * output_component_image,
      cv::Mat * output_component_mask,
      cv::Mat1b * output_ellipse_mask) const;

  static void normalize_jovian_image(cv::InputArray _src, cv::InputArray mask,
      cv::OutputArray dst,
      double sigma);

  static double compute_jovian_derotation_cost(const cv::Mat1f & reference_component_image,
      const cv::Mat1f & curret_rotated_component_image,
      const cv::Mat1f & rotation_mask,
      cv::Mat1f * difference_image = nullptr) ;

protected:

  cv::RotatedRect reference_ellipse_;
  cv::RotatedRect current_ellipse_;
  cv::Rect reference_boundig_box_;
  cv::Rect current_boundig_box_;
  cv::Mat reference_component_image_;
  cv::Mat current_component_image_;
  cv::Mat reference_component_mask_;
  cv::Mat current_component_mask_;

  cv::Mat reference_normalized_image_;
  cv::Mat current_normalized_image_;
  cv::Mat1b reference_ellipse_mask_;
  cv::Mat1b current_ellipse_mask_;
  double normalization_scale_ = 1.0;

  cv::Mat2f current_total_remap_;
  cv::Mat1f current_total_mask_;
  cv::Mat1b current_total_binary_mask_;

  c_ecc_forward_additive ecc_;
  c_ecc_pyramide_align ecch_;
  c_ecch_flow eccflow_;

  double min_rotation_ = -30 * CV_PI / 180;
  double max_rotation_ = +30 * CV_PI / 180;
  int eccflow_support_scale_ = 3;
  int eccflow_normalization_scale_ = 0;
  int eccflow_max_pyramid_level_ = 1;

  //bool align_jovian_disk_horizontally_ = false;


  std::string debug_path_;
  bool enable_debug_ = false;

};

#endif /* __jupiter_h__ */
