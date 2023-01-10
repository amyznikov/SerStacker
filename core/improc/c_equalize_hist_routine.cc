/*
 * c_equalize_hist_routine.cc
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#include "c_equalize_hist_routine.h"


c_equalize_hist_routine::c_class_factory c_equalize_hist_routine::class_factory;

c_equalize_hist_routine::c_equalize_hist_routine(bool enabled) :
    base(&class_factory, enabled)
{
}

c_equalize_hist_routine::ptr c_equalize_hist_routine::create(bool enabled)
{
  return ptr (new this_class(enabled));
}


bool c_equalize_hist_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.depth() == CV_8U ) {

    const int cn = image.channels();
    if( cn == 1 ) {
      cv::equalizeHist(image.getMat(), image);
    }
    else {
      std::vector<cv::Mat> channels;
      cv::split(image, channels);

      for( int i = 0; i < cn; ++i ) {
        cv::equalizeHist(channels[i], channels[i]);
      }

      cv::merge(channels, image);
    }
  }
  return true;
}

bool c_equalize_hist_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  return true;
}

bool c_equalize_hist_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  return true;
}
