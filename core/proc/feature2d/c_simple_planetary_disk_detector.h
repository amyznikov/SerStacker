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

  c_simple_planetary_disk_detector(double gbsigma = 1, int se_radius = 5);

  static cv::Ptr<this_class> create(double gbsigma = 1, int se_radius = 5);

  void detect(cv::InputArray _src, std::vector<cv::KeyPoint> & keypoints,
      cv::InputArray _mask) override;

  void compute( cv::InputArray /*image*/,
      CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors) override;

  void detectAndCompute(cv::InputArray image, cv::InputArray mask,
      CV_OUT std::vector<cv::KeyPoint> & keypoints,
      cv::OutputArray descriptors,
      bool useProvidedKeypoints = false) override;

protected:
  double _gsigma;
  int _se_radius = 5;
  cv::Rect _component_rect;
};

#endif /* __c_simple_planetary_disk_detector_h__ */
