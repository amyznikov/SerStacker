/*
 * cv2qt.cc
 *
 *  Created on: Mar 8, 2020
 *      Author: amyznikov
 */

#include "cv2qt.h"
#include <core/debug.h>

/*
 * Create flow BGR image using HSV color space
 *  flow: 2-channel input flow matrix
 *  dst : output BRG image
 *
 *  The code is extracted from OpenCV example dis_opticalflow.cpp
 * */
bool flow2HSV(cv::InputArray flow, cv::Mat & dst, double maxmotion, bool invert_y)
{
  if ( flow.channels() != 2 ) {
    CF_FATAL("Unsupported number of channels: %d. 2-channel image expected", flow.channels());
    return false;
  }

  cv::Mat uv[2], mag, ang;

  cv::split(flow.getMat(), uv);

  if ( flow.depth() == CV_32F || flow.depth() == CV_64F ) {
    if ( invert_y ) {
      cv::multiply(uv[1], -1, uv[1]);
    }
  }
  else {
    uv[0].convertTo(uv[0], CV_32F);
    uv[1].convertTo(uv[1], CV_32F, invert_y ? -1 : 1);
  }

  cv::cartToPolar(uv[0], uv[1], mag, ang, true);

  if ( maxmotion > 0 ) {
    cv::threshold(mag, mag, maxmotion, maxmotion, cv::THRESH_TRUNC);
  }

  cv::normalize(mag, mag, 0, 1, cv::NORM_MINMAX);

  const cv::Mat hsv[3] = {
      ang,
      mag,
      cv::Mat::ones(ang.size(), ang.type())
  };

  cv::merge(hsv, 3, dst);
  cv::cvtColor(dst, dst, cv::COLOR_HSV2RGB);
  dst.convertTo(dst, CV_8U, 255);

  return true;
}


bool cv2qt(cv::InputArray __src, QImage * dst, bool rgbswap)
{
  cv::Mat src;

  if ( __src.channels() == 2 ) {  // treat as optical flow marrix
    flow2HSV(__src, src, 0, true);
    rgbswap = false;
  }
  else if ( __src.depth() == CV_8U ) {
    src = __src.getMat();
  }
  else {
    cv::normalize(__src, src, 0, 255, cv::NORM_MINMAX);
    src.convertTo(src, CV_8U);
  }

  switch ( src.type() ) {
  case CV_8UC1 :
    if ( dst->format() != QImage::Format_Grayscale8 || dst->width() != src.cols
        || dst->height() != src.rows ) {
      *dst = QImage(src.cols, src.rows, QImage::Format_Grayscale8);
    }
    for ( int i = 0; i < src.rows; ++i ) {
      memcpy(dst->scanLine(i), src.ptr<const uint8_t>(i), src.cols);
    }
    return true;

  case CV_8UC3 :
    if ( dst->format() != QImage::Format_RGB888 || dst->width() != src.cols
        || dst->height() != src.rows ) {
      *dst = QImage(src.cols, src.rows, QImage::Format_RGB888);
    }

    if ( !rgbswap ) {
      for ( int i = 0; i < src.rows; ++i ) {
        memcpy(dst->scanLine(i), src.ptr<const uint8_t>(i), src.cols * 3);
      }
    }
    else {
      for ( int y = 0; y < src.rows; ++y ) {
        const cv::Vec3b * srcp = src.ptr<const cv::Vec3b>(y);
        cv::Vec3b * dstp = (cv::Vec3b *) dst->scanLine(y);
        for ( int x = 0; x < src.cols; ++x ) {
          dstp[x].val[0] = srcp[x].val[2];
          dstp[x].val[1] = srcp[x].val[1];
          dstp[x].val[2] = srcp[x].val[0];
        }
      }
    }
    return true;

  case CV_8UC4 :
    if ( dst->format() != QImage::Format_ARGB32 || dst->width() != src.cols
        || dst->height() != src.rows ) {
      *dst = QImage(src.cols, src.rows, QImage::Format_ARGB32);
    }
    for ( int i = 0; i < src.rows; ++i ) {
      memcpy(dst->scanLine(i), src.ptr<const uint8_t>(i), src.cols * 4);
    }
    return true;

  default :
    CF_FATAL("Unhandled cv2 type: depth=%d channels=%d", src.depth(), src.channels());
    break;
  }

  return false;
}
