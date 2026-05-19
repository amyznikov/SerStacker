/*
 * c_simple_planetary_disk_detector.h
 *
 *  Created on: Jun 5, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_simple_planetary_disk_detector_h__
#define __c_simple_planetary_disk_detector_h__

#include <opencv2/opencv.hpp>

class c_simple_planetary_disk_detector :
    public cv::Feature2D
{
public:
  typedef c_simple_planetary_disk_detector this_class;
  typedef cv::Feature2D bae;

  c_simple_planetary_disk_detector(double gbsigma = 1, double stdev_factor = 0.5, int se_close_radius = 2);

  static cv::Ptr<this_class> create(double gbsigma = 1, double stdev_factor = 0.5, int se_close_radius = 2);

  void detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints,
      cv::InputArray _mask) override;


protected:
  double gbsigma_;
  double stdev_factor_ = 0.5;
  int se_close_radius_ = 2;
  cv::Rect component_rect_;

  //cv::Mat cmponent_mask_;
  //cv::Point2f geometrical_center_;
  //cv::Mat debug_image;
};

#endif /* __c_simple_planetary_disk_detector_h__ */
