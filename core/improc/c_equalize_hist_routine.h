/*
 * c_equalize_hist_routine.h
 *
 *  Created on: Jan 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_equalize_hist_routine_h__
#define __c_equalize_hist_routine_h__

#include "c_image_processor.h"
#include <core/proc/pixtype.h>

class c_equalize_hist_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_equalize_hist_routine,
      "equalize_hist", "Calls cv::equalizeHist()");

  bool deserialize(c_config_setting settings)
  {
    if ( !base::deserialize(settings) ) {
      return false;
    }
    return true;
  }

  bool serialize(c_config_setting settings) const
  {
    if ( !base::serialize(settings) ) {
      return false;
    }
    return true;
  }

  bool process(cv::InputOutputArray _image, cv::InputOutputArray mask)
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

};

#endif /* __c_equalize_hist_routine_h__ */
