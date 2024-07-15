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
  double stdev_factor = 0.5;
  int se_close_radius = 3;
};

class c_saturn_ellipse_detector
{
public:

  c_saturn_ellipse_detector_options & options()
  {
    return options_;
  }

  const c_saturn_ellipse_detector_options & options() const
  {
    return options_;
  }

  const cv::Mat1b & saturn_mask() const
  {
    return saturn_mask_;
  }

  const cv::RotatedRect & saturn_bounding_box() const
  {
    return saturn_bounding_box_;
  }

  //const cv::Mat & gray_image() const;


  bool detect(cv::InputArray _image,
      cv::InputArray mask = cv::noArray());


protected:
  c_saturn_ellipse_detector_options options_;
  cv::Mat1b saturn_mask_;
  cv::RotatedRect saturn_bounding_box_;
};

#endif /* __c_saturn_ellipse_detector_h__ */
