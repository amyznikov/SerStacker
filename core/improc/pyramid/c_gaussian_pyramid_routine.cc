/*
 * c_gaussian_pyramid_routine.cc
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#include "c_gaussian_pyramid_routine.h"

void c_gaussian_pyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "count",  ctx(&this_class::_count), "");
   ctlbind(ctls, "borderType",  ctx(&this_class::_borderType), "");
}

bool c_gaussian_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _count);
    SERIALIZE_OPTION(settings, save, *this, _borderType);
    return true;
  }
  return false;
}

bool c_gaussian_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( _count > 0 ) {
    for( int i = 0; i < _count && std::min(image.cols(), image.rows()) > 3; ++i ) {
      cv::pyrDown(image, image, cv::Size(), _borderType);
      if( !mask.empty() ) {
        cv::pyrDown(mask, mask, cv::Size(), _borderType);
      }
    }
  }
  else if( _count < 0 ) {
    for( int i = 0; i < - _count && std::max(image.cols(), image.rows()) < 16000; ++i ) {
      cv::pyrUp(image, image, cv::Size(), _borderType);
      if( !mask.empty() ) {
        cv::pyrUp(mask, mask, cv::Size(), _borderType);
      }
    }
  }

  return true;
}
