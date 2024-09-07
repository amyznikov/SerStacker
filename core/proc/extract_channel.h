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
  color_channel_gray = color_channel_featured_begin + 1, // cv::cvtColor(cv::COLOR_BGR2GRAY)
  color_channel_luminance = color_channel_featured_begin + 2,  // cv::cvtColor(cv::COLOR_BGR2GRAY) -> cv::extractChannel(0)
  color_channel_red = color_channel_featured_begin + 3, // cv::extractChannel(2)
  color_channel_green = color_channel_featured_begin + 4, // cv::extractChannel(1)
  color_channel_blue = color_channel_featured_begin + 5, // cv::extractChannel(0)
  color_channel_min_inensity = color_channel_featured_begin + 6, // cv::reduce(min)
  color_channel_max_intensity = color_channel_featured_begin + 7, // cv::reduce(max)
  color_channel_avg_intensity = color_channel_featured_begin + 8, // cv::reduce(avg)
  color_channel_max_color = color_channel_featured_begin + 9, // cv::reduce(max) - cv::reduce(min)
  color_channel_sum_intensity = color_channel_featured_begin + 10, // cv::reduce(sum)
};


// Extract requested color channel form input color image
bool extract_channel(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray srcmsk, cv::OutputArray dstmsk,
    int channel, // channel index or enum color_channel_type
    double output_scale = 1.,
    int output_depth = -1,
    double output_depth_scale = 1.0);



#endif /* __extract_channel_h__ */
