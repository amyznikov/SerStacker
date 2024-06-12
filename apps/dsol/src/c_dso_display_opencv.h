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
    public dso::c_dso_display
{
public:
  typedef c_dso_display_opencv this_class;
  typedef dso::c_dso_display base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  using FrameHessian = dso::FrameHessian;
  using CalibHessian = dso::CalibHessian;
  using c_image_and_exposure = dso::c_image_and_exposure;
  using c_frame_shell = dso::c_frame_shell;

  void set_enable_display(bool v);
  bool enable_display() const;

  // max frames in window.
  void set_max_frames(int v);
  int max_frames() const;

  bool needDisplayInputFrame() const override;
  void displayInputFrame(const c_image_and_exposure & image, int id) override;

  bool needDisplayTrackedFrame() const override;
  void displayTrackedFrame(const FrameHessian * /*fh*/) override;

  bool needDisplaySelectorImage() const override;
  void displaySelectorImage(const FrameHessian * /*fh*/) override;
  bool needDisplayResImage() const override;
  void displayResImage(const cv::Mat & image) override;
  bool needDisplayDepthImageFloat() const override;
  void displayDepthImageFloat(const cv::Mat& image, const FrameHessian * KF) override;
  bool needDisplayCameraPose() const override;
  void displayCameraPose(const c_frame_shell * frame, const CalibHessian * HCalib) override;
  bool needDisplayGraph() const override;
  void displayGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i> > > & connectivity) override;
  bool needDisplayKeyframe() const override;
  void displayKeyframe(const FrameHessian* frame, bool _final, const CalibHessian * HCalib) override;
  void pushLiveFrame(const FrameHessian * image) override;
  void pushDepthImage(const cv::Mat & image) override;
  bool needPushDepthImage() override;

  //void pushLiveFrame(const dso::FrameHessian * fh) override;

protected:

  void displayImage(const std::string & windowName, const cv::Mat & img, bool autoSize = false);
  void displayImageStitch(const std::string & windowName, const std::vector<cv::Mat> & images, int cc = 0, int rc = 0);
  int waitKey(int milliseconds) ;

protected:
  std::unordered_set<std::string> open_windows_;
  int max_frames_ = 7; // max frames in window.
  bool enable_display_ = true;
};

#endif /* __c_output_wrapper_opencv_h__ */
