/*
 * c_jovian_derotation2.h
 *
 *  Created on: Aug 28, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_jovian_derotation2_h__
#define __c_jovian_derotation2_h__

#include <opencv2/opencv.hpp>
#include "c_jovian_ellipse_detector2.h"

class c_jovian_derotation2
{
public:

  bool detect(cv::InputArray reference_image, cv::InputArray reference_mask = cv::noArray());
  bool compute(double zrotation_deg);
  bool compute(double current_tstamp_sec, double target_tstamp_sec);

  const c_jovian_ellipse_detector2_options & detector_options() const
  {
    return _detector.options();
  }

  c_jovian_ellipse_detector2_options & detector_options()
  {
    return _detector.options();
  }

  bool draw_detected(cv::InputOutputArray image)
  {
    return _detector.draw_detected(image);
  }

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
    return _detector.current_remap();
  }

  const cv::Mat1b & current_bmask() const
  {
    return _detector.current_bmask();
  }

  const cv::Mat1f & current_wmask() const
  {
    return _detector.current_wmask();
  }

  const cv::Mat1f & pcax() const
  {
    return _detector.pcax();
  }

  const cv::Mat1f & pcay() const
  {
    return _detector.pcay();
  }

protected:
  c_jovian_ellipse_detector2 _detector;
//  cv::Size _reference_image_size;
//  cv::Mat2f _current_remap;
//  cv::Mat1b _current_bmask;
//  cv::Mat1f _current_wmask;

};

#endif /* __c_jovian_derotation2_h__ */
