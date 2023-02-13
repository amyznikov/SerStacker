/*
 * c_align_color_channels_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_align_color_channels_routine.h"
#include <core/proc/reduce_channels.h>


bool c_align_color_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !enable_threshold_ ) {
    return algorithm_.align(reference_channel_, image, image, mask, mask);
  }

  cv::Mat smask;

  cv::compare(image, threshold_, smask, cv::CMP_GE);
  if ( smask.channels() > 1 ) {
    reduce_color_channels(smask, cv::REDUCE_MAX);
  }
  if ( !mask.empty() ) {
    cv::bitwise_and(mask, smask, smask);
  }

  return algorithm_.align(reference_channel_, image, image, smask);
}

