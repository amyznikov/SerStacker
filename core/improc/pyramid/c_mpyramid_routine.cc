/*
 * c_mpyramid_routine.cc
 *
 *  Created on: Aug 8, 2023
 *      Author: amyznikov
 */

#include "c_mpyramid_routine.h"
#include <core/proc/laplacian_pyramid.h>

void c_mpyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "minimum_image_size", ctx(&this_class::_minimum_image_size), "");
  ctlbind(ctls, "display_pos", ctx(&this_class::_display_pos), "");
}

bool c_mpyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _minimum_image_size);
    SERIALIZE_OPTION(settings, save, *this, _display_pos);
    return true;
  }
  return false;
}

bool c_mpyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  std::vector<cv::Mat> layers;

  if( !build_mpyramid(image, layers, _minimum_image_size) ) {
    CF_ERROR("build_mpyramid() fails");
    return false;
  }

  const int display_layer =
      std::max(0, std::min(_display_pos, (int) (layers.size()) - 1));

  // CF_DEBUG("display_layer=%d layers.size=%zu", display_layer, layers.size());
  layers[display_layer].copyTo(image);

  if( mask.needed() ) {
    mask.release();
  }

  return true;
}
