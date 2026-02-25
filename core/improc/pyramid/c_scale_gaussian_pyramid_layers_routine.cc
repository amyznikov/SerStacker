/*
 * c_scale_gaussian_pyramid_layers_routine.cc
 *
 *  Created on: Jul 13, 2022
 *      Author: amyznikov
 */

#include "c_scale_gaussian_pyramid_layers_routine.h"
#include <numeric>

void c_scale_gaussian_pyramid_layers_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "scales", ctx(&this_class::_scales), "");
  ctlbind(ctls, "borderType", ctx(&this_class::_borderType), "");
}

bool c_scale_gaussian_pyramid_layers_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _scales);
    SERIALIZE_OPTION(settings, save, *this, _borderType);
    return true;
  }
  return false;
}

bool c_scale_gaussian_pyramid_layers_routine::process(cv::InputOutputArray _image, cv::InputOutputArray mask)
{
  if( _scales.size() < 2 ) {
    return true; //
  }

  cv::Mat & image =
      _image.getMatRef();

  const double norm =
      std::accumulate(_scales.begin(), _scales.end(), 0.);

  std::vector<cv::Mat> layers;
  std::vector<cv::Size> layer_sizes;

  static float k[3] = { 0.25, 0.5, 0.25 };
  static const cv::Mat1f K(1, 3, k);

  layers.resize(_scales.size());
  layer_sizes.resize(_scales.size());

  image.copyTo(layers[0]);
  cv::sepFilter2D(layers[0], layers[1], -1, K, K, cv::Point(-1, -1), 0, _borderType);
  for( int i = 1, n = layers.size() - 1; i < n; ++i ) {
    cv::pyrDown(layers[i], layers[i + 1], cv::Size(), _borderType);
  }

  for( int i = 0, n = layers.size(); i < n; ++i ) {
    layer_sizes[i] = layers[i].size();
  }

  for( int i = 2, n = layers.size(); i < n; ++i ) {
    for( int j = i - 1; j > 0; --j ) {
      cv::pyrUp(layers[i], layers[i], layer_sizes[j], _borderType);
    }
  }

  image.setTo(0);

  for( int i = 0, n = layers.size(); i < n; ++i ) {

    if( _scales[i] != 0 ) {

      if( i < n - 1 ) {
        cv::subtract(layers[i], layers[i + 1], layers[i]);
      }

      cv::scaleAdd(layers[i], _scales[i] / norm, image, image);
    }
  }

  return true;
}
