/*
 * c_epipolar_matcher.h
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_epipolar_matcher_h__
#define __c_epipolar_matcher_h__

#include <opencv2/opencv.hpp>
#include <core/proc/array2d.h>
#include <core/settings/opencv_settings.h>



struct c_epipolar_matcher_options
{
  int max_disparity = 100;
  int diff_threshold = 10;
  int avg_scale = 3;
  bool enabled = false;
  bool enable_debug = false;
};

class c_epipolar_matcher
{
public:

  c_epipolar_matcher();
  c_epipolar_matcher(const c_epipolar_matcher_options & opts);

  bool enabled() const;
  void set_enabled(bool v);

  void set_debug_path(const std::string & v);
  const std::string & debug_path() const;

  void set_options(const c_epipolar_matcher_options & opts);
  const c_epipolar_matcher_options & options() const;
  c_epipolar_matcher_options & options();


  bool match(cv::InputArray current_image, cv::InputArray current_mask,
      cv::InputArray previous_image, cv::InputArray previous_mask,
      const cv::Matx33f & derotation_homography,
      const cv::Point2d & epipole_location);

  const cv::Mat2f & matches() const;
  const cv::Mat2i & back_matches() const;
  const cv::Mat1w & costs() const;

  bool serialize(c_config_setting settings, bool save);

protected:
  c_epipolar_matcher_options options_;
  std::string debug_path_;
  cv::Mat2f matches_;
  cv::Mat2i back_matches_;
  cv::Mat1w costs_;
};


cv::Mat1f matchesToEpipolarDisparity(const cv::Mat2f & matches,
    const cv::Point2d & epipole_position);



#endif /* __c_epipolar_matcher_h__ */
