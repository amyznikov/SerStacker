/*
 * c_set_pixels_routine.cpp
 *
 *  Created on: Mar 21, 2026
 *      Author: amyznikov
 */

#include "c_set_pixels_routine.h"

void c_set_pixels_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "value", ctx(&this_class::_value), "cv::Scalar(b;g;r;a)");
  ctlbind(ctls, "_invertMask", ctx(&this_class::_invertMask), "Set true to invert input mask betfore apply");
}

bool c_set_pixels_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _value);
    SERIALIZE_OPTION(settings, save, *this, _invertMask);
//    SERIALIZE_OPTION(settings, save, *this, _input_channel);
//    SERIALIZE_OPTION(settings, save, *this, _output_channel);
    return true;
  }
  return false;

}

bool c_set_pixels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( image.empty() || mask.empty() ) {
    return true;
  }

  if( mask.channels() == 1 ) {
    const cv::Mat srcm = mask.getMat();
    image.setTo(_value, _invertMask ? ~srcm : srcm);
  }
  else if( image.channels() == mask.channels() ) {

    std::vector<cv::Mat> channels;
    std::vector<cv::Mat> mchannels;

    cv::split(image, channels);
    cv::split(_invertMask ? ~mask.getMat() : mask.getMat(), channels);

    for( int c = 0, cn = image.channels(); c < cn; ++c ) {
      channels[c].setTo(_value, mchannels[c]);
    }

    cv::merge(channels, image);
  }

  return true;
}
