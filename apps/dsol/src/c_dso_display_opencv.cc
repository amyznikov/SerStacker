/*
 * c_output_wrapper_opencv.cc
 *
 *  Created on: Jun 11, 2024
 *      Author: amyznikov
 */

#include "c_dso_display_opencv.h"
#include <opencv2/highgui/highgui.hpp>
#include "dso/util/globalCalib.h"
#include "dso/HessianBlocks.h"
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

void c_dso_display_opencv::createStitch()
{
  const cv::Size size(wG[0], hG[0]);
  const cv::Size stitch_size(3 * wG[0], 3 * hG[0]);

  if ( stitch_image.size() != size ) {

    stitch_image.create(stitch_size);

    InputFrameDisplay = stitch_image(cv::Rect(0,0, size.width, size.height));
    SelectorImageDisplay = stitch_image(cv::Rect(size.width,0, size.width, size.height));
    KeyframeDisplay = stitch_image(cv::Rect(2 * size.width, 0, size.width, size.height));
  }

}

int c_dso_display_opencv::waitKey(int milliseconds)
{
  if( !enable_display_ ) {
    return 0;
  }

  return cv::waitKey(milliseconds);
}

void c_dso_display_opencv::displayFrame(const cv::Mat & image, cv::Mat3b & target_pane)
{
  if ( image.type() == CV_8UC3 ) {
    image.copyTo(target_pane);
  }
  else if ( image.channels() == 3 ) {
    image.convertTo(target_pane, CV_8U);
  }
  else {
    cv::Mat tmp;
    image.convertTo(tmp, CV_8U);
    cv::cvtColor(tmp, target_pane, cv::COLOR_GRAY2BGR);
  }

  displayStitch();
}

void c_dso_display_opencv::displayStitch()
{
  cv::imshow("dso", stitch_image);
  cv::waitKey(10);
}

bool c_dso_display_opencv::needDisplayInputFrame() const
{
  return true;
}


void c_dso_display_opencv::displayInputFrame(const c_image_and_exposure & frame, int id)
{
  createStitch();
  displayFrame(frame.image(), InputFrameDisplay);
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


void c_dso_display_opencv::displaySelectorImage(const FrameHessian * fh, const float * map_out)
{
  createStitch();

  const cv::Size size =
      SelectorImageDisplay.size();

  for( int y = 0; y < size.height; ++y ) {
    for( int x = 0; x < size.width; ++x ) {

      const int i =
          x + y * size.width;

      const float c =
          std::min(255.f, fh->dI[i][0] * 0.7f);

      SelectorImageDisplay[y][x] =
          cv::Vec3b(c, c, c);

    }
  }

  for( int y = 0; y < size.height; ++y ) {
    for( int x = 0; x < size.width; ++x ) {

      const int i =
          x + y * size.width;

      switch ((int) map_out[i]) {
        case 1:
          cv::rectangle(SelectorImageDisplay, cv::Point(x - 1, y - 1), cv::Point(x + 1, y + 1),
              cv::Scalar(0, 255, 0));
          break;
        case 2:
          cv::rectangle(SelectorImageDisplay, cv::Point(x - 1, y - 1), cv::Point(x + 1, y + 1),
              cv::Scalar(255, 0, 0));
          break;
        case 4:
          cv::rectangle(SelectorImageDisplay, cv::Point(x - 1, y - 1), cv::Point(x + 1, y + 1),
              cv::Scalar(0, 0, 255));
          break;
      }
    }
  }

  displayStitch();
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
  if ( _final ) {
    const cv::Size size(wG[0], hG[0]);
    displayFrame(cv::Mat3f(size.height, size.width, (cv::Vec3f*) frame->dI), KeyframeDisplay);
  }
}

void c_dso_display_opencv::pushLiveFrame(const FrameHessian * image)
{
  CF_DEBUG("c_dso_display_opencv: ");
}


void c_dso_display_opencv::pushDepthImage(const cv::Mat & image)
{
  CF_DEBUG("c_dso_display_opencv: ");
}

bool c_dso_display_opencv::needPushDepthImage() const
{
  return true;
}


