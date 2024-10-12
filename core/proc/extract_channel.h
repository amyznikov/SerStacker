/*
 * extract_channel.h
 *
 *  Created on: Jan 14, 2020
 *      Author: amyznikov
 */

#ifndef __extract_channel_h__
#define __extract_channel_h__

#include <opencv2/opencv.hpp>

enum color_channel_type
{
  color_channel_unknown = -1,

  color_channel_0 = 0,
  color_channel_1,
  color_channel_2,
  color_channel_3,
  color_channel_4,

  color_channel_featured_begin = 100,

  color_channel_dont_change = color_channel_featured_begin + 0, // return back original image

  color_channel_red, // cv::extractChannel(2)
  color_channel_green, // cv::extractChannel(1)
  color_channel_blue, // cv::extractChannel(0)

  color_channel_gray, // cv::cvtColor(cv::COLOR_BGR2GRAY)
  color_channel_luminance_YCrCb,  // cv::cvtColor(cv::COLOR_BGR2YCRCB) -> cv::extractChannel(0)
  color_channel_luminance_lab,  // cv::cvtColor(cv::COLOR_BGR2Lab) -> cv::extractChannel(0)
  color_channel_luminance_luv,  // cv::cvtColor(cv::COLOR_BGR2Luv) -> cv::extractChannel(0)
  color_channel_luminance_hsv,  // cv::cvtColor(cv::COLOR_BGR2HSV) -> cv::extractChannel(2)
  color_channel_luminance_hls,  // cv::cvtColor(cv::COLOR_BGR2HLS) -> cv::extractChannel(1)

  color_channel_min_inensity, // cv::reduce(min)
  color_channel_max_intensity, // cv::reduce(max)
  color_channel_avg_intensity, // cv::reduce(avg)
  color_channel_sum_intensity, // cv::reduce(sum)
  color_channel_absmax, // max of absolute value
  color_channel_first_nonzero, //

  color_channel_max_color, // cv::reduce(max) - cv::reduce(min)
  color_channel_max_gradient,

};


// Extract requested color channel form input color image
bool extract_channel(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray srcmsk, cv::OutputArray dstmsk,
    int channel, // channel index or enum color_channel_type
    double output_scale = 1.,
    int output_depth = -1,
    double output_depth_scale = 1.0);



#endif /* __extract_channel_h__ */
