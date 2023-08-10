/*
 * c_mpyramid_routine.cc
 *
 *  Created on: Aug 8, 2023
 *      Author: amyznikov
 */

#include "c_mpyramid_routine.h"
#include <core/proc/laplacian_pyramid.h>


void c_mpyramid_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, min_image_size, "Specify minimum image size");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, display_pos, "Specify display node position");
}

bool c_mpyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, min_image_size);
    SERIALIZE_PROPERTY(settings, save, *this, display_pos);
    return true;
  }
  return false;
}

bool c_mpyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  std::vector<cv::Mat> layers;

  if( !build_mpyramid(image, layers, minimum_image_size_) ) {
    CF_ERROR("build_mpyramid() fails");
    return false;
  }

  const int display_layer =
      std::max(0, std::min(display_pos_, (int) (layers.size()) - 1));

  // CF_DEBUG("display_layer=%d layers.size=%zu", display_layer, layers.size());
  layers[display_layer].copyTo(image);

  if ( mask.needed() ) {
    mask.release();
  }

  return true;
}
