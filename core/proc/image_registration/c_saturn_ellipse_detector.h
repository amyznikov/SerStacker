/*
 * c_saturn_ellipse_detector.h
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_saturn_ellipse_detector_h__
#define __c_saturn_ellipse_detector_h__

#include <opencv2/opencv.hpp>

struct c_saturn_ellipse_detector_options
{
  double gbsigma = 1; // optional gaussian blur input image for planetary detection
  double stdev_factor = 0.5; // stdev threshold for planetary detection

  double equatorial_radius = 130; //  for manual setup only, [pix]
  double ring_radius = 250; // for manual setup only, [pix]

  // planet inclination to the ray of view, for manual setup, [deg]
  cv::Vec3d pose =
      cv::Vec3d(90, 0, 0);

  // planet center on image, for manual setup only, [pix]
  cv::Point2f center =
      cv::Point2f (-1, -1);

  int se_close_radius = 3;

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


    bool show_ring = true;
    bool show_smask = false;
    bool show_sbox = false;
    bool print_debug_info = true;

  } draw;
};

class c_saturn_ellipse_detector
{
public:

  bool detect(cv::InputArray image, cv::InputArray mask = cv::noArray());

  bool draw_detected(cv::InputOutputArray image) const;


  c_saturn_ellipse_detector_options & options()
  {
    return _options;
  }

  const c_saturn_ellipse_detector_options & options() const
  {
    return _options;
  }

  const cv::Mat1b & saturn_mask() const
  {
    return _saturn_mask;
  }

  const cv::RotatedRect & saturn_bounding_box() const
  {
    return _saturn_bounding_box;
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

protected:
  c_saturn_ellipse_detector_options _options;
  cv::Mat1b _saturn_mask;
  cv::RotatedRect _saturn_bounding_box;
  cv::RotatedRect _planetary_disk_ellipse;
  cv::Mat1b _planetary_disk_ellipse_mask;

  double _ring_radius = 0;
  cv::Point2f _center;

  cv::Vec3d _axes;  // [pix]
  cv::Vec3d _pose; // [radian]


  bool _saturn_detected = false;


};

#endif /* __c_saturn_ellipse_detector_h__ */
