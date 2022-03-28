/*
 * c_color_saturation_routine.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#include "c_color_saturation_routine.h"
#include <core/proc/color_saturation.h>
#include <core/io/save_image.h>

c_color_saturation_routine::c_class_factory c_color_saturation_routine::class_factory;

c_color_saturation_routine::c_color_saturation_routine(bool enabled)
  : base(&class_factory, enabled)
{
}

c_color_saturation_routine::ptr c_color_saturation_routine::create(bool enabled)
{
  return ptr(new this_class(enabled));
}

c_color_saturation_routine::ptr c_color_saturation_routine::create(const std::vector<double> & scales, bool enabled)
{
  ptr obj(new this_class(enabled));
  obj->set_scales(scales);
  return obj;
}

void c_color_saturation_routine::set_scales(const std::vector<double> & scales)
{
  scales_ = scales;
}

const std::vector<double> & c_color_saturation_routine::scales() const
{
  return scales_;
}


bool c_color_saturation_routine::deserialize(c_config_setting settings)
{
  if ( !base::deserialize(settings) ) {
    return false;
  }

  LOAD_PROPERTY(settings, this, scales);

  return true;
}

bool c_color_saturation_routine::serialize(c_config_setting settings) const
{
  if ( !base::serialize(settings) ) {
    return false;
  }

  SAVE_PROPERTY(settings, *this, scales);

  return true;
}

bool c_color_saturation_routine::process(cv::InputOutputArray _image, cv::InputOutputArray mask)
{
  if( _image.channels() != 3 || scales_.empty() ) {
    return true; //
  }

  cv::Mat & image = _image.getMatRef();

  if( scales_.size() == 1 ) {
    return color_saturation_hls(image, scales_[0], mask);
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

