/*
 * c_vlo_depth_segmentation_routine.cc
 *
 *  Created on: Nov 16, 2023
 *      Author: amyznikov
 */

#include "c_vlo_depth_segmentation_routine.h"

void c_vlo_depth_segmentation_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, min_distance, "min_distance");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_distance, "max_distance");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, vlo_walk_error, "vlo_walk_error");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, min_slope, "min_slope");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_slope, "max_slope");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, min_pts, "min_pts");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, output_type, "output_type");

}

bool c_vlo_depth_segmentation_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, min_distance);
    SERIALIZE_PROPERTY(settings, save, *this, max_distance);
    SERIALIZE_PROPERTY(settings, save, *this, vlo_walk_error);
    SERIALIZE_PROPERTY(settings, save, *this, min_slope);
    SERIALIZE_PROPERTY(settings, save, *this, max_slope);
    SERIALIZE_PROPERTY(settings, save, *this, min_pts);
    SERIALIZE_PROPERTY(settings, save, *this, output_type);
    return true;
  }
  return false;
}

bool c_vlo_depth_segmentation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  if ( !vlo_depth_segmentation(image.getMat(), image, opts_) ) {
    CF_ERROR("vlo_depth_segmentation() fails");
    return false;
  }

  return true;
}
