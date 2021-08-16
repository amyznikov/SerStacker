/*
 * color_saturation.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#include "color_saturation.h"
#include <tbb/tbb.h>


bool color_saturation_hls(cv::InputArray src, cv::OutputArray dst, double scale,
    cv::InputArray srcmask)
{

  if ( src.channels() != 3 ) { // Not a BGR
    src.copyTo(dst);
    return false;
  }

  cv::Mat hls;
  cv::Mat channels[3];

  cv::cvtColor(src, hls, cv::COLOR_BGR2HLS_FULL);
  cv::split(hls, channels);
  cv::multiply(channels[2], scale, channels[2]);
  if ( channels[2].depth() >= CV_32F ) {
    cv::min(channels[2], 1, channels[2]);
  }
  cv::merge(channels, 3, hls);
  cv::cvtColor(hls, dst, cv::COLOR_HLS2BGR_FULL);

  return true;
}

