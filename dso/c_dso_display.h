/*
 * c_dso_display.h
 *
 *  Created on: Jun 11, 2024
 *      Author: amyznikov
 *
 *  This file is derived from DSO sources
 */

#pragma once
#ifndef __c_dso_display_h__
#define __c_dso_display_h__

#include "c_image_and_exposure.h"
#include "util/NumType.h"
#include <map>
#include <memory>

namespace dso {

class FrameHessian;
class CalibHessian;
class c_frame_shell;

class c_dso_display
{
public:
  typedef c_dso_display this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  virtual ~c_dso_display()
  {
  }

  virtual bool needDisplayInputFrame() const
  {
    return false;
  }

  virtual void displayInputFrame(const c_image_and_exposure & image, int id)
  {
  }

  virtual bool needDisplayTrackedFrame() const
  {
    return false;
  }

  virtual void displayTrackedFrame(const FrameHessian * /*fh*/)
  {
  }

  virtual bool needDisplayKeyframe() const
  {
    return false;
  }

  virtual void displayKeyframe(const FrameHessian* frame, bool _final, const CalibHessian * HCalib)
  {
  }

  virtual bool needDisplaySelectorImage() const
  {
    return false;
  }

  virtual void displaySelectorImage(const FrameHessian * /*fh*/, const float * /*map_out*/ )
  {
  }

  virtual bool needDisplayResImage() const
  {
    return false;
  }

  virtual void displayResImage(const cv::Mat & image)
  {
  }

  virtual bool needDisplayDepthImageFloat() const
  {
    return false;
  }

  virtual void displayDepthImageFloat(const cv::Mat& image, const FrameHessian * KF)
  {
  }


  virtual bool needDisplayCameraPose() const
  {
    return false;
  }

  virtual void displayCameraPose(const c_frame_shell * frame, const CalibHessian * HCalib)
  {
  }

  virtual bool needDisplayGraph() const
  {
    return false;
  }

  virtual void displayGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
      Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i> > > & connectivity)
  {
  }


  virtual void pushLiveFrame(const FrameHessian * image)
  {
  }

  virtual void pushDepthImage(const cv::Mat & image)
  {
  }

  virtual bool needPushDepthImage()
  {
    return false;
  }


};


constexpr int keypoint_display_radius = 3;
}


#endif /* __c_dso_display_h__ */
