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
    cv::RotatedRect * output_ellipse_rect);



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
      cv::Mat2f & rmap,
      cv::InputArray input_mask = cv::noArray());

  const cv::RotatedRect & reference_ellipse() const;
  const cv::Rect & reference_ellipse_boudig_box() const;


protected:
  cv::RotatedRect reference_ellipse_;
  cv::Rect reference_ellipse_boudig_box_;
  cv::Mat reference_image_;
  cv::Mat reference_mask_;
};

#endif /* __jupiter_h__ */
