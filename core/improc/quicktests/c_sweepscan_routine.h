/*
 * c_sweepscan_routine.h
 *
 *  Created on: Apr 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sweepscan_routine_h__
#define __c_sweepscan_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/stereo/c_sweepscan_stereo_matcher.h>

class c_sweepscan_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_sweepscan_routine,
      "sweepscan", "c_sweepscan_routine");

  // matcher
  using OutputType = c_sweepscan_stereo_matcher::OutputType;

  void set_output_type(OutputType v);
  OutputType output_type() const;

  void set_max_disparity(int v);
  int max_disparity() const;

  void set_disp12maxDiff(int v);
  int disp12maxDiff() const;

  void set_texture_threshold(int v);
  int texture_threshold() const;

  void set_max_scale(int v);
  int max_scale() const;

  void set_debug_directory(const std::string & v);
  const std::string & debug_directory() const;

  void set_debug_points(const std::vector<cv::Point> & v);
  const std::vector<cv::Point>& debug_points() const;

  // routine
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_sweepscan_stereo_matcher sm_;
  cv::Mat images_[2];
  cv::Mat masks_[2];
  cv::Mat1w m;
  cv::Mat1b mm;
};

#endif /* __c_sweepscan_routine_h__ */

