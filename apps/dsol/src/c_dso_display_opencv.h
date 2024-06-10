/*
 * c_output_wrapper_opencv.h
 *
 *  Created on: Jun 11, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_output_wrapper_opencv_h__
#define __c_output_wrapper_opencv_h__

#include "dso/c_dso_display.h"
#include <unordered_set>

class c_dso_display_opencv :
    public dso::c_dso_display2d
{
public:
  typedef c_dso_display_opencv this_class;
  typedef dso::c_dso_display2d base;

  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  void set_enable_display(bool v);
  bool enable_display() const;

  // max frames in window.
  void set_max_frames(int v);
  int max_frames() const;

  void displayImage(const std::string & windowName, const cv::Mat & img, bool autoSize = false) override;
  void displayImageStitch(const std::string & windowName, const std::vector<cv::Mat> & images, int cc = 0, int rc = 0) override;
  int waitKey(int milliseconds) override;

protected:
  std::unordered_set<std::string> open_windows_;
  int max_frames_ = 7; // max frames in window.
  bool enable_display_ = true;
};

#endif /* __c_output_wrapper_opencv_h__ */
