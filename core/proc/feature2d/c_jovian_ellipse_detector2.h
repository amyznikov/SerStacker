/*
 * c_jovian_ellipse_detector2.h
 *
 *  Created on: Aug 23, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_jovian_ellipse_detector2_h__
#define __c_jovian_ellipse_detector2_h__

#include <opencv2/opencv.hpp>

struct c_jovian_ellipse_detector2_options
{
  double gbsigma = 1; // optional gaussian blur input image for planetary detection
  double stdev_factor = 0.25; // stdev threshold for planetary detection
  double pca_blur = 3;
  int se_close_radius = 3;

  double equatorial_radius = 130; //  for manual setup only, [pix]

  // planet inclination to the ray of view, for manual setup, [deg]
  cv::Vec3d pose =
      cv::Vec3d(87, 0, 0);

  // planet center on image, for manual setup only, [pix]
  cv::Point2f center =
      cv::Point2f (-1, -1);

  // optional offset to planet center on image, mainly for auto location, [pix]
  cv::Point2f offset =
      cv::Point2f(0, 0);

  bool auto_location = true;

  struct draw_options
  {
    double latidute_step = 30; // [deg]
    double longitude_step = 30; // [deg]
    double deltat = 0; // [sec]

    cv::Scalar line_color =
        cv::Scalar::all(255);

    cv::Scalar outline_color =
        cv::Scalar::all(255);


    bool show_smask = false;
    bool show_bmask = false;
    bool show_wmask = false;
    bool show_sbox = false;
    bool show_pcax = false;
    bool show_pcay = false;
    bool print_debug_info = true;

  } draw;
};

class c_jovian_ellipse_detector2
{
public:

  bool detect(cv::InputArray image, cv::InputArray mask = cv::noArray());

  bool draw_detected(cv::InputOutputArray image);

  bool compute_derotation_for_angle(double zrotation_deg);
  bool compute_derotation_for_time(double deltat_sec);

  c_jovian_ellipse_detector2_options & options()
  {
    return _options;
  }

  const c_jovian_ellipse_detector2_options & options() const
  {
    return _options;
  }

  const cv::RotatedRect & planetary_disk_ellipse() const
  {
    return _planetary_disk_ellipse;
  }

  const cv::Mat1b & planetary_disk_ellipse_mask() const
  {
    return _planetary_disk_ellipse_mask;
  }

  const cv::Point2f & center() const
  {
    return _center;
  }

  const cv::Vec3d & axes() const
  {
    return _axes;
  }

  const cv::Vec3d & pose() const
  {
    return _pose;
  }

  const cv::Mat2f & current_remap() const
  {
    return _current_remap;
  }

  const cv::Mat1b & current_bmask() const
  {
    return _current_bmask;
  }

  const cv::Mat1f & current_wmask() const
  {
    return _current_wmask;
  }

  const cv::Mat1f & pcax() const
  {
    return _pca_gx;
  }

  const cv::Mat1f & pcay() const
  {
    return _pca_gy;
  }

protected:
  double compute_jovian_orientation_pca(const cv::Mat & gray_image, const cv::Mat & planetary_disk_mask,
      const cv::Rect & component_rect);

protected:
  c_jovian_ellipse_detector2_options _options;
  cv::Size _reference_image_size;
  cv::RotatedRect _planetary_disk_ellipse;
  cv::Mat1b _planetary_disk_ellipse_mask;
  cv::Mat1b _detected_planetary_disk_edge;
  cv::Mat1b _final_planetary_disk_mask;
  cv::Mat2f _current_remap;
  cv::Mat1b _current_bmask;
  cv::Mat1f _current_wmask;

  //cv::RotatedRect _final_planetary_disk_ellipse;
  cv::Mat1f _pca_gx;
  cv::Mat1f _pca_gy;

  cv::Point2f _center;
  cv::Vec3d _axes;  // [pix]
  cv::Vec3d _pose; // [radian]

  bool _detected = false;
};

#endif /* __c_jovian_ellipse_detector2_h__ */
