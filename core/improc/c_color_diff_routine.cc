/*
 * c_color_diff_routine.cc
 *
 *  Created on: Apr 7, 2023
 *      Author: amyznikov
 */

#include "c_color_diff_routine.h"
#include <core/proc/reduce_channels.h>

bool c_color_diff_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.channels() == 1 ) {
    image.setTo(0);
  }
  else {
    std::vector<cv::Mat> channels;
    cv::Mat avg;

    reduce_color_channels(image, avg, cv::REDUCE_AVG, CV_32F);
    cv::split(image, channels);

    for( int i = 0, cn = channels.size(); i < cn; ++i ) {

      if ( channels[i].depth() < CV_32F ) {
        channels[i].convertTo(channels[i], CV_32F);
      }

      cv::subtract(channels[i], avg, channels[i]);
    }

    cv::merge(channels, image);
  }

  return true;
}
