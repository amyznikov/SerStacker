/*
 * c_histogram_white_balance_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_histogram_white_balance_routine.h"
#include <core/proc/white_balance/histogram_white_balance.h>
#include <core/proc/reduce_channels.h>


bool c_histogram_white_balance_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( image.channels() < 2 ) {
    return true;
  }

  cv::Mat objmask;

  if ( !enable_threshold_ ) {
    objmask = mask.getMat();
  }
  else {
    cv::compare(image, threshold_, objmask, cv::CMP_GE);

    if ( objmask.channels() > 1 ) {
      reduce_color_channels(objmask, cv::REDUCE_MIN);
    }

    if ( !mask.empty() ) {
      cv::bitwise_and(mask, objmask, objmask);
    }
  }

  return histogram_white_balance(image.getMatRef(),
      objmask,
      image.getMatRef(),
      lclip_,
      hclip_);

}


