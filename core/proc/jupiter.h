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

struct c_detect_jovian_ellipse_debug_images {
  cv::Mat gray_image;
  cv::Mat component_mask;
  cv::Mat cropped_component_mask;
  cv::Mat cropped_component_image;
  cv::Mat initial_artifical_ellipse;
  cv::Mat fitted_artifical_ellipse;
};

bool detect_jovian_ellipse(cv::InputArray _image,
    cv::RotatedRect * output_ellipse_rect,
    const std::string & debug_path = "",
    c_detect_jovian_ellipse_debug_images * debug = nullptr,
    const std::vector<float> * hlines = nullptr);



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
  double normalization_blur = 2;
  double gradient_blur = 2;
  int normalization_scale = 3;
};

class c_jovian_ellipse_detector
{
public:
  void set_hlines(const std::vector<float> & v);
  const std::vector<float> & hlines() const;

  void set_normalization_scale(int v);
  int normalization_scale() const;
  int actual_normalization_scale() const;

  void set_normalization_blur(double v);
  double normalization_blur() const;

  void set_gradient_blur(double v);
  double gradient_blur() const;

  void set_options(const c_jovian_ellipse_detector_options & v);
  c_jovian_ellipse_detector_options * options();

  bool detect_planetary_disk(cv::InputArray _image, cv::InputArray mask = cv::noArray());

  // ellipse in source image coordinates
  const cv::RotatedRect & planetary_disk_ellipse() const;
  const cv::Mat & uncropped_planetary_disk_mask() const;

  // bounding box for component_mask
  const cv::Rect & crop_bounding_box() const;

  const cv::Mat & cropped_gray_image() const ;
  const cv::Mat & cropped_normalized_image() const ;
  const cv::Mat & cropped_gradient_image() const ;

  const cv::Mat & initial_artificial_ellipse() const ;
  const cv::Mat & initial_artificial_ellipse_fit() const ;
  const cv::Mat & cropped_final_ellipse_fit() const ;

protected:
  c_jovian_ellipse_detector_options options_;
  int actual_normalization_scale_ = -1;

  cv::RotatedRect ellipse_;
  cv::Mat uncropped_planetary_disk_mask_;

  cv::Rect crop_bounding_box_;
  cv::Mat cropped_gray_image_;
  cv::Mat cropped_normalized_image_;
  cv::Mat cropped_gradient_image_;

  cv::Mat cropped_artificial_ellipse_;
  cv::Mat cropped_initial_ellipse_fit_;
  cv::Mat cropped_final_ellipse_fit_;
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

  // set mininum rotation angle for best rotation search range.
  void set_min_rotation(double v);
  double min_rotation() const;

  // set maximum rotation angle for best rotation search range.
  void set_max_rotation(double v);
  double max_rotation() const;

  void set_normalization_scale(int v);
  int normalization_scale() const;

  void set_normalization_blur(double v);
  double normalization_blur() const;

  void set_gradient_blur(double v);
  double gradient_blur() const;

  void set_eccflow_support_scale(int v);
  int eccflow_support_scale() const;

  void set_eccflow_normalization_scale(int v);
  int eccflow_normalization_scale() const;

  void set_eccflow_max_pyramid_level(int v);
  int max_eccflow_pyramid_level(int v);

  void set_hlines(const std::vector<float> & hlines);
  const std::vector<float> & hlines() const;

  //  void set_align_jovian_disk_horizontally(bool v);
  //  bool align_jovian_disk_horizontally() const;

  const cv::RotatedRect & reference_ellipse() const;
  const cv::RotatedRect & current_ellipse() const;

  const cv::Rect & reference_bounding_box() const;
  const cv::Rect & current_bounding_box() const;

  const cv::Mat & reference_uncropped_planetary_disk_mask() const ;
  const cv::Mat & current_uncropped_planetary_disk_mask() const ;

  const cv::Mat2f & current_cropped_derotation_remap() const;
  const cv::Mat1f & current_cropped_wmask() const;

  void set_debug_path(const std::string & v);
  const std::string & debug_path() const;

protected:
  static double compute_jovian_derotation_cost(const cv::Mat1f & reference_component_image,
      const cv::Mat1f & curret_rotated_component_image,
      const cv::Mat1f & rotation_mask,
      cv::Mat1f * difference_image = nullptr) ;

protected:

  cv::RotatedRect reference_ellipse_;
  cv::RotatedRect current_ellipse_;
  cv::Rect reference_bounding_box_;
  cv::Rect current_bounding_box_;
  cv::Mat reference_cropped_gray_image_;
  cv::Mat current_cropped_gray_image_;
  cv::Mat reference_uncropped_planetary_disk_mask_;
  cv::Mat current_uncropped_planetary_disk_mask_;

  cv::Mat reference_cropped_normalized_image_;
  cv::Mat current_cropped_normalized_image_;

  cv::Mat1f current_cropped_wmask_;
  cv::Mat2f current_cropped_remap_;

//  c_ecc_forward_additive ecc_;
//  c_ecch ecch_;
  c_ecch_flow eccflow_;
  c_jovian_ellipse_detector planetary_detector_;

  double min_rotation_ = -80 * CV_PI / 180;
  double max_rotation_ = +80 * CV_PI / 180;
  int eccflow_support_scale_ = 4;
  int eccflow_normalization_scale_ = 2;
  int eccflow_max_pyramid_level_ = 0;

  std::string debug_path_;
};




#endif /* __jupiter_h__ */
