/*
 * c_equalize_hist_routine.cc
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#include "c_equalize_hist_routine.h"

void c_equalize_hist_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
}

bool c_equalize_hist_routine::process(cv::InputOutputArray _image, cv::InputOutputArray mask)
{
  cv::Mat image;

  if( _image.depth() == CV_8U ) {
    image = _image.getMat();
  }
  else {
    _image.getMat().convertTo(image, CV_8U,
        255. / get_maxval_for_pixel_depth(_image.depth()));
  }

  const int cn = image.channels();
  if( cn == 1 ) {
    cv::equalizeHist(image, _image);
  }
  else {

    std::vector<cv::Mat> channels;
    cv::split(image, channels);

    for( int i = 0; i < cn; ++i ) {
      cv::equalizeHist(channels[i], channels[i]);
    }

    cv::merge(channels, _image);
  }

  return true;
}

