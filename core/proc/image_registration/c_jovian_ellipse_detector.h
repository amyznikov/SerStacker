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

struct c_jovian_ellipse_detector_options
{
  double stdev_factor = 0.5;
  double pca_blur = 3;
  cv::Point2f offset;
};

class c_jovian_ellipse_detector
{
public:

  void set_enable_debug_images(bool v);
  bool enable_debug_images() const;

  void set_stdev_factor(double v);
  double stdev_factor() const;

  void set_pca_blur(double sigma);
  double pca_blur() const;

  void set_offset(const cv::Point2f & v);
  const cv::Point2f & offset() const;

  void set_options(const c_jovian_ellipse_detector_options & v);
  const c_jovian_ellipse_detector_options & options() const;
  c_jovian_ellipse_detector_options & options();

  bool detect_jovian_disk(cv::InputArray _image,
      cv::InputArray mask = cv::noArray());

  const cv::Mat & gray_image() const;
  const cv::Mat1f pca_gx() const;
  const cv::Mat1f pca_gy() const;

  const cv::Mat1b & detected_planetary_disk_mask() const;
  const cv::Mat1b & detected_planetary_disk_edge() const;
  const cv::Mat1b & final_planetary_disk_mask() const;

  //const cv::RotatedRect & ellipseAMS() const;
  const cv::RotatedRect & final_planetary_disk_ellipse() const;

protected:
  double compute_jovian_orientation_pca(const cv::Mat & gray_image, const cv::Mat & planetary_disk_mask,
      const cv::Rect & component_rect);

protected:
  c_jovian_ellipse_detector_options options_;
  bool enable_debug_images_ = false;

  cv::Mat gray_image_;
  cv::Mat1b detected_planetary_disk_mask_;
  cv::Mat1b detected_planetary_disk_edge_;
  cv::Mat1b final_planetary_disk_mask_;
  cv::Mat1f pca_gx_;
  cv::Mat1f pca_gy_;

  //cv::RotatedRect ellipseAMS_;
  cv::RotatedRect final_planetary_disk_ellipse_;
};

cv::Rect compute_ellipse_bounding_box(const cv::RotatedRect & rc);
cv::Rect compute_ellipse_crop_box(const cv::RotatedRect & rc, const cv::Size & total_image_size, int margin = -1);

#endif /* __c_jovian_ellipse_detector_h__ */
