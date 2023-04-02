/*
 * c_color_saturation_routine.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#include "c_color_saturation_routine.h"
#include <core/proc/color_saturation.h>

bool c_color_saturation_routine::process(cv::InputOutputArray _image, cv::InputOutputArray mask)
{
  cv::Mat & image =
      _image.getMatRef();

  if( image.channels() != 3 || scales_.empty() ) {
    return true; //
  }


  if( scales_.size() == 1 ) {

    return color_saturation_hls(image,
        scales_[0],
        mask);
  }

  std::vector<cv::Mat> layers;
  cv::Mat tmp;

  cv::buildPyramid(image, layers, scales_.size() - 1);

  for( int i = 0, n = layers.size(); i < n; ++i ) {

    if( i < n - 1 ) {
      cv::pyrUp(layers[i + 1], tmp, layers[i].size());
      cv::subtract(layers[i], tmp, layers[i]);
    }

    color_saturation_hls(layers[i], scales_[i]);
  }

  for( int i = layers.size() - 1; i > 0; --i ) {

    cv::pyrUp(layers[i], layers[i], layers[i - 1].size());

    cv::add(layers[i], layers[i - 1], layers[i - 1]);
  }

  return true;
}

