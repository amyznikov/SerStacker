/*
 * extract_channel.h
 *
 *  Created on: Jan 14, 2020
 *      Author: amyznikov
 */

#ifndef __extract_channel_h__
#define __extract_channel_h__

#include <opencv2/opencv.hpp>

enum color_channel_type {

  channel_channel_featured_begin = 100,

  color_channel_unknown = -1,
  color_channel_gray = channel_channel_featured_begin + 0, // cv::cvtColor(cv::COLOR_BGR2GRAY)
  color_channel_luminance = channel_channel_featured_begin + 1,  // cv::cvtColor(cv::COLOR_BGR2GRAY) -> cv::extractChannel(0)
  color_channel_red = channel_channel_featured_begin + 2, // cv::extractChannel(2)
  color_channel_green = channel_channel_featured_begin + 3, // cv::extractChannel(1)
  color_channel_blue = channel_channel_featured_begin + 4, // cv::extractChannel(0)
  color_channel_min_inensity = channel_channel_featured_begin + 5, // cv::reduce(min)
  color_channel_max_inensity = channel_channel_featured_begin + 6, // cv::reduce(max)
  color_channel_avg_inensity = channel_channel_featured_begin + 7, // cv::reduce(avg)
};


const extern struct color_channel_type_desc {
  const char * name;
  enum color_channel_type value;
} color_channel_types[];

std::string toStdString(enum color_channel_type v);
enum color_channel_type fromStdString(const std::string  & s,
    enum color_channel_type defval );



// Extract requested color channel form input color image
bool extract_channel(cv::InputArray src, cv::OutputArray dst,
    cv::InputArray srcmsk, cv::OutputArray dstmsk,
    int channel, // channel index or enum color_channel_type
    double output_scale = 1.,
    int output_depth = -1,
    double output_depth_scale = 1.0);



#endif /* __extract_channel_h__ */
