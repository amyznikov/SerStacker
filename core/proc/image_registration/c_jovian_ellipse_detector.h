/*
 * c_jovian_ellipse_detector.h
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_jovian_ellipse_detector_h__
#define __c_jovian_ellipse_detector_h__

#include <opencv2/opencv.hpp>

struct c_jovian_ellipse_detector_options {
  double stdev_factor = 0.5;
  bool force_reference_ellipse = false;
};

class c_jovian_ellipse_detector
{
public:

  void set_enable_debug_images(bool v);
  bool enable_debug_images() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  void set_options(const c_jovian_ellipse_detector_options & v);
  const c_jovian_ellipse_detector_options & options() const;
  c_jovian_ellipse_detector_options & options();

  bool detect_jovian_disk(cv::InputArray _image,
      cv::InputArray mask = cv::noArray());

  const cv::Mat & gray_image() const ;
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

  const cv::Mat & gradient_test_image() const
  {
    return gradient_test_image_;
  }


protected:
  c_jovian_ellipse_detector_options options_;
  bool enable_debug_images_ = false;

  cv::Mat gray_image_;

  cv::Mat gradient_test_image_;


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
};

cv::Rect compute_ellipse_bounding_box(const cv::RotatedRect & rc);
cv::Rect compute_ellipse_crop_box(const cv::RotatedRect & rc, const cv::Size & total_image_size, int margin = -1);

#endif /* __c_jovian_ellipse_detector_h__ */
