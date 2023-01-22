/*
 * c_gaussian_pyramid_routine.cc
 *
 *  Created on: Jul 13, 2022
 *      Author: amyznikov
 */

#include "c_gaussian_pyramid_routine.h"
#include <numeric>

bool c_gaussian_pyramid_routine::process(cv::InputOutputArray _image, cv::InputOutputArray mask)
{
  if( scales_.size() < 2 ) {
    return true; //
  }

  cv::Mat & image =
      _image.getMatRef();

  const double norm =
      std::accumulate(scales_.begin(), scales_.end(), 0.);

  std::vector<cv::Mat> layers;
  std::vector<cv::Size> layer_sizes;

  static float k[3] = { 0.25, 0.5, 0.25 };
  static const cv::Mat1f K(1, 3, k);

  layers.resize(scales_.size());
  layer_sizes.resize(scales_.size());

  image.copyTo(layers[0]);
  cv::sepFilter2D(layers[0], layers[1], -1, K, K, cv::Point(-1, -1), 0, borderType_);
  for( int i = 1, n = layers.size() - 1; i < n; ++i ) {
    cv::pyrDown(layers[i], layers[i + 1], cv::Size(), borderType_);
  }

  for( int i = 0, n = layers.size(); i < n; ++i ) {
    layer_sizes[i] = layers[i].size();
  }

  for( int i = 2, n = layers.size(); i < n; ++i ) {
    for( int j = i - 1; j > 0; --j ) {
      cv::pyrUp(layers[i], layers[i], layer_sizes[j], borderType_);
    }
  }

  image.setTo(0);

  for( int i = 0, n = layers.size(); i < n; ++i ) {

    if( scales_[i] != 0 ) {

      if( i < n - 1 ) {
        cv::subtract(layers[i], layers[i + 1], layers[i]);
      }

      cv::scaleAdd(layers[i], scales_[i] / norm, image, image);
    }
  }


  return true;
}
