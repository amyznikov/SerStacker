/*
 * c_test_scale_space_routine.cc
 *
 *  Created on: Sep 2, 2023
 *      Author: amyznikov
 */

#include "c_test_scale_space_routine.h"


void c_test_scale_space_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "minimum_image_size",  ctx(&this_class::_minimum_image_size), "");
}

bool c_test_scale_space_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _minimum_image_size);
    return true;
  }
  return false;
}

bool c_test_scale_space_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  std::vector<cv::Mat> laplacians;
  cv::Mat current_image, downscalled_image, laplacian;

  laplacians.reserve(12);

  current_image = image.getMat().clone();

  while ( 42 ) {

    const cv::Size current_size =
        current_image.size();

    const cv::Size next_size((current_size.width + 1) / 2, (current_size.height + 1) / 2);
    if( std::min(next_size.width, next_size.height) <= _minimum_image_size ) {
      break;
    }

    cv::pyrDown(current_image, downscalled_image, next_size);
    cv::pyrUp(downscalled_image, laplacian, current_size);

  }

  return false;
}
