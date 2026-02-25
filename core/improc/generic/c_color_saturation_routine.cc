/*
 * c_color_saturation_routine.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#include "c_color_saturation_routine.h"
#include <core/proc/color_saturation.h>

void c_color_saturation_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "scales", ctx(&this_class::_scales), "");
}

bool c_color_saturation_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _scales);
    return true;
  }
  return false;
}

bool c_color_saturation_routine::process(cv::InputOutputArray _image, cv::InputOutputArray mask)
{
  cv::Mat & image =
      _image.getMatRef();

  if( image.channels() != 3 || _scales.empty() ) {
    return true; //
  }


  if( _scales.size() == 1 ) {

    return color_saturation_hls(image,
        _scales[0],
        mask);
  }

  std::vector<cv::Mat> layers;
  cv::Mat tmp;

  cv::buildPyramid(image, layers, _scales.size() - 1);

  for( int i = 0, n = layers.size(); i < n; ++i ) {

    if( i < n - 1 ) {
      cv::pyrUp(layers[i + 1], tmp, layers[i].size());
      cv::subtract(layers[i], tmp, layers[i]);
    }

    color_saturation_hls(layers[i], _scales[i]);
  }

  for( int i = layers.size() - 1; i > 0; --i ) {

    cv::pyrUp(layers[i], layers[i], layers[i - 1].size());

    cv::add(layers[i], layers[i - 1], layers[i - 1]);
  }

  return true;
}

