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

#include <opencv2/opencv.hpp>
#include "util/NumType.h"
#include <map>
#include <memory>

namespace dso {

class FrameHessian;
class CalibHessian;
class c_frame_shell;

class c_dso_display2d
{
public:
  typedef c_dso_display2d this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  virtual ~c_dso_display2d()
  {
  }

  virtual void displayImage(const std::string & windowName, const cv::Mat & img, bool autoSize = false)
  {
  }

  virtual void displayImageStitch(const std::string & windowName, const std::vector<cv::Mat> & images, int cc = 0, int rc = 0)
  {
  }

  virtual int waitKey(int milliseconds)
  {
    return 0;
  }

};

class c_dso_display3d
{
public:
  typedef c_dso_display3d this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  virtual ~c_dso_display3d()
  {
  }

  /*  Usage:
   *  Called once after each new Keyframe is inserted & optimized.
   *  [connectivity] contains for each frame-frame pair the number of [0] active residuals in between them,
   *  and [1] the number of marginalized reisduals between them.
   *  frame-frame pairs are encoded as HASH_IDX = [(int)hostFrameKFID << 32 + (int)targetFrameKFID].
   *  the [***frameKFID] used for hashing correspond to the [FrameHessian]->frameID.
   *
   *  Calling:
   *  Always called, no overhead if not used.
   */
  virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
          Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i> > > & connectivity)
  {
  }

  /* Usage:
   * Called after each new Keyframe is inserted & optimized, with all keyframes that were part of the active window during
   * that optimization in [frames] (with final=false). Use to access the new frame pose and points.
   * Also called just before a frame is marginalized (with final=true) with only that frame in [frames]; at that point,
   * no further updates will ever occur to it's optimized values (pose & idepth values of it's points).
   *
   * If you want always all most recent values for all frames, just use the [final=false] calls.
   * If you only want to get the final model, but don't care about it being delay-free, only use the
   * [final=true] calls, to save compute.
   *
   * Calling:
   * Always called, negligible overhead if not used.
   */
  virtual void publishKeyframes(std::vector<FrameHessian*> & frames, bool _final, CalibHessian * HCalib)
  {
  }

  /* Usage:
   * Called once for each tracked frame, with the real-time, low-delay frame pose.
   *
   * Calling:
   * Always called, no overhead if not used.
   */
  virtual void publishCamPose(c_frame_shell * frame, CalibHessian * HCalib)
  {
  }

  /* Usage:
   * Called once for each new frame, before it is tracked (i.e., it doesn't have a pose yet).
   *
   * Calling:
   * Always called, no overhead if not used.
   */
  virtual void pushLiveFrame(FrameHessian * image)
  {
  }

  /* called once after a new keyframe is created, with the color-coded, forward-warped inverse depthmap for that keyframe,
   * which is used for initial alignment of future frames. Meant for visualization.
   *
   * Calling:
   * Needs to prepare the depth image, so it is only called if [needPushDepthImage()] returned true.
   */
  virtual void pushDepthImage(const cv::Mat & image)
  {
  }

  virtual bool needPushDepthImage()
  {
    return false;
  }

  /* Usage:
   * called once after a new keyframe is created, with the forward-warped inverse depthmap for that keyframe.
   * (<= 0 for pixels without inv. depth value, >0 for pixels with inv. depth value)
   *
   * Calling:
   * Always called, almost no overhead if not used.
   */
  virtual void pushDepthImageFloat(const cv::Mat&, FrameHessian * KF)
  {
  }

  /* call on finish */
  virtual void join()
  {
  }

  /* call on reset */
  virtual void reset()
  {
  }
};

class c_dso_display
{
public:
  typedef c_dso_display this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  void emplace_back(c_dso_display2d * display)
  {
    if ( display ) {
      display2d.emplace_back(display);
    }
  }

  void emplace_back(c_dso_display3d * display)
  {
    if ( display ) {
      display3d.emplace_back(display);
    }
  }

  void displayImage(const std::string & windowName, const cv::Mat & image, bool autoSize = false) const
  {
    for ( auto * d : display2d ) {
      d->displayImage(windowName, image, autoSize);
    }
  }

  void displayImageStitch(const std::string & windowName, const std::vector<cv::Mat> & images, int cc = 0, int rc = 0) const
  {
    for ( auto * d : display2d ) {
      d->displayImageStitch(windowName, images, cc, rc);
    }
  }

  // FIXME: get rid of this
  int waitKey(int milliseconds) const
  {
    int key = 0;
    for( auto * d : display2d ) {
      if( (key = d->waitKey(milliseconds)) ) {
        break;
      }
    }
    return key;
  }


  void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
      Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i> > > & connectivity) const
  {
    for ( auto * d : display3d ) {
      d->publishGraph(connectivity);
    }
  }

  void publishKeyframes(std::vector<FrameHessian*> & frames, bool _final, CalibHessian * HCalib) const
  {
    for ( auto * d : display3d ) {
      d->publishKeyframes(frames, _final, HCalib);
    }
  }

  void publishCamPose(c_frame_shell * frame, CalibHessian * HCalib) const
  {
    for ( auto * d : display3d ) {
      d->publishCamPose(frame, HCalib);
    }
  }

  void pushLiveFrame(FrameHessian * image) const
  {
    for ( auto * d : display3d ) {
      d->pushLiveFrame(image);
    }
  }

  void pushDepthImage(const cv::Mat & image) const
  {
    for ( auto * d : display3d ) {
      d->pushDepthImage(image);
    }
  }

  bool needPushDepthImage() const
  {
    for( auto * d : display3d ) {
      if( d->needPushDepthImage() ) {
        return true;
      }
    }
    return false;
  }

  void pushDepthImageFloat(const cv::Mat& image, FrameHessian * KF) const
  {
    for ( auto * d : display3d ) {
      d->pushDepthImageFloat(image, KF);
    }
  }

  /* call on finish */
  void join() const
  {
    for ( auto * d : display3d ) {
      d->join();
    }
  }

  /* call on reset */
  void reset() const
  {
    for ( auto * d : display3d ) {
      d->reset();
    }
  }

protected:
  std::vector<c_dso_display2d*> display2d;
  std::vector<c_dso_display3d*> display3d;
};


constexpr int keypoint_display_radius = 3;
}


#endif /* __c_dso_display_h__ */
