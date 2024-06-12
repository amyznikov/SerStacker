/*
 * c_output_wrapper_opencv.cc
 *
 *  Created on: Jun 11, 2024
 *      Author: amyznikov
 */

#include "c_dso_display_opencv.h"
#include <opencv2/highgui/highgui.hpp>
#include <core/debug.h>

using namespace dso;

void c_dso_display_opencv::set_enable_display(bool v)
{
  enable_display_ = v;
}

bool c_dso_display_opencv::enable_display() const
{
  return enable_display_;
}

void c_dso_display_opencv::set_max_frames(int v)
{
  max_frames_ = v;
}

int c_dso_display_opencv::max_frames() const
{
  return max_frames_;
}


void c_dso_display_opencv::displayImage(const std::string & windowName, const cv::Mat & image, bool autoSize)
{
  if( !enable_display_ ) {
    return;
  }

  if( !autoSize ) {

    if( open_windows_.find(windowName) == open_windows_.end() ) {
      cv::namedWindow(windowName, cv::WINDOW_NORMAL);
      cv::resizeWindow(windowName, image.cols, image.rows);
      open_windows_.insert(windowName);
    }
  }

  cv::Mat display;

  if ( image.depth() == CV_8U ) {
      display = image;
  }
  else {
    image.convertTo(display, CV_8U);
  }

  CF_DEBUG("display: %dx%d", display.cols, display.rows);

  cv::imshow(windowName, display);
  cv::waitKey(20);

}

void c_dso_display_opencv::displayImageStitch(const std::string & windowName, const std::vector<cv::Mat> & images, int cc, int rc)
{
  if( images.empty() || !enable_display_ ) {
    return;
  }

  // get dimensions.
  const int w =
      images[0].cols;

  const int h =
      images[0].rows;

  const int num =
      std::max((int) max_frames_,
          (int) images.size());

  // get optimal dimensions.
  int bestCC = 0;
  float bestLoss = 1e10;

  for( int cc = 1; cc < 10; cc++ ) {

    const int ww =
        w * cc;

    const int hh =
        h * ((num + cc - 1) / cc);

    const float wLoss =
        ww / 16.0f;

    const float hLoss =
        hh / 10.0f;

    float loss =
        std::max(wLoss, hLoss);

    if( loss < bestLoss ) {
      bestLoss = loss;
      bestCC = cc;
    }
  }

  int bestRC =
      ((num + bestCC - 1) / bestCC);

  if( cc != 0 ) {
    bestCC = cc;
    bestRC = rc;
  }

  cv::Mat stitch(bestRC * h, bestCC * w,
      images[0].type(),
      cv::Scalar::all(0));

  for( int i = 0, n = (int)images.size(); i < n && i < bestCC * bestRC; ++i ) {

    const int r =
        i / bestCC;

    const int c =
        i % bestCC;

    cv::Mat roi =
        stitch(cv::Rect(c * w, r * h, w, h));

    images[i].copyTo(roi);
  }

  displayImage(windowName, stitch,
      false);
}


int c_dso_display_opencv::waitKey(int milliseconds)
{
  if( !enable_display_ ) {
    return 0;
  }

  return cv::waitKey(milliseconds);
}


bool c_dso_display_opencv::needDisplayInputFrame() const
{
  return true;
}

void c_dso_display_opencv::displayInputFrame(const c_image_and_exposure & image, int id)
{
  CF_DEBUG("c_dso_display_opencv: id=%d", id);

  displayImage("input frame", image.image(), false);
}

bool c_dso_display_opencv::needDisplayTrackedFrame() const
{
  return true;
}

void c_dso_display_opencv::displayTrackedFrame(const FrameHessian * /*fh*/)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needDisplaySelectorImage() const
{
  return true;
}


void c_dso_display_opencv::displaySelectorImage(const FrameHessian * /*fh*/)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needDisplayResImage() const
{
  return true;
}


void c_dso_display_opencv::displayResImage(const cv::Mat & image)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needDisplayDepthImageFloat() const
{
  return true;
}


void c_dso_display_opencv::displayDepthImageFloat(const cv::Mat& image, const FrameHessian * KF)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needDisplayCameraPose() const
{
  return true;
}


void c_dso_display_opencv::displayCameraPose(const c_frame_shell * frame, const CalibHessian * HCalib)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needDisplayGraph() const
{
  return true;
}


void c_dso_display_opencv::displayGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i> > > & connectivity)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needDisplayKeyframe() const
{
  return true;
}


void c_dso_display_opencv::displayKeyframe(const FrameHessian* frame, bool _final, const CalibHessian * HCalib)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

void c_dso_display_opencv::pushLiveFrame(const FrameHessian * image)
{
  CF_DEBUG("c_dso_display_opencv: ");
}


void c_dso_display_opencv::pushDepthImage(const cv::Mat & image)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needPushDepthImage()
{
  return true;
}


