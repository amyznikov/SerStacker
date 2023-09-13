/*
 * c_radial_scale_routine.cc
 *
 *  Created on: Sep 12, 2023
 *      Author: amyznikov
 */

#include "c_radial_scale_routine.h"
#include <core/proc/stereo/scale_sweep.h>
#include <core/debug.h>

void c_radial_scale_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, reference_point, "Reference point location X,Y [px]");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, disparity, "disparity for image scale calculation");
}

bool c_radial_scale_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, reference_point);
    SERIALIZE_PROPERTY(settings, save, *this, disparity);
    return true;
  }
  return false;

}

bool c_radial_scale_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( (image.needed() && !image.empty()) || (mask.needed() && !mask.empty()) ) {

    const cv::Size image_size =
        image.empty() ?
            mask.size() :
            image.size();

    if( rmap_.size() != image_size ) {
      create_scale_compression_remap(disparity_, image_size, reference_point_, rmap_);
    }

    if( image.needed() && !image.empty() ) {
      cv::remap(image.getMat(), image, rmap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }

    if( mask.needed() ) {

      if( !mask.empty() ) {
        cv::remap(mask.getMat(), mask, rmap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      }
      else {
        cv::remap(cv::Mat1b(image_size, 255), mask, rmap_, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      }

      cv::compare(mask, 255, mask, cv::CMP_GE);
    }

  }

  return true;
}
