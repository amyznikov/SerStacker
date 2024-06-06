/*
 * flow2HSV.cc
 *
 *  Created on: Jun 6, 2024
 *      Author: amyznikov
 */
#include "flow2HSV.h"
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
    cv::threshold(mag, mag, maxmotion, maxmotion,
        cv::THRESH_TRUNC);
  }

  cv::normalize(mag, mag, 0, 1,
      cv::NORM_MINMAX);

  const cv::Mat hsv[3] = {
      ang,
      mag,
      mag/*cv::Mat::ones(ang.size(), ang.type())*/
  };

  cv::merge(hsv, 3, dst);
  cv::cvtColor(dst, dst, cv::COLOR_HSV2RGB);
  //cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
  dst.convertTo(dst, CV_8U, 255);

  return true;
}
