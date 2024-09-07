/*
 * c_saturn_derotation.h
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_saturn_derotation_h__
#define __c_saturn_derotation_h__

#include <opencv2/opencv.hpp>
#include "c_saturn_ellipse_detector.h"
// #include "ecc2.h"

class c_saturn_derotation
{
public:
  void set_detector_options(const c_saturn_ellipse_detector_options & v);
  const c_saturn_ellipse_detector_options & detector_options() const;

  bool detect(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool compute(double zrotation_deg);

  bool compute(double current_tstamp_sec, double target_tstamp_sec);

  const cv::RotatedRect & planetary_disk_ellipse() const
  {
    return _detector.planetary_disk_ellipse();
  }

  const cv::Mat & planetary_disk_ellipse_mask() const
  {
    return _detector.planetary_disk_ellipse_mask();
  }

  const cv::Mat2f & current_derotation_remap() const
  {
    return _current_remap;
  }

  const cv::Mat1f & current_wmask() const
  {
    return _current_wmask;
  }


protected:
  c_saturn_ellipse_detector _detector;
  cv::Size _reference_image_size;

  cv::Mat2f _current_remap;
  cv::Mat1f _current_wmask;

};

#endif /* __c_saturn_derotation_h__ */
