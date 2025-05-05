/*
 * c_seed_fill_segmentation_routine.cc
 *
 *  Created on: May 5, 2025
 *      Author: amyznikov
 */

#include "c_seed_fill_segmentation_routine.h"
#include <core/debug.h>

void c_seed_fill_segmentation_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, threshold, "Threshold value for color difference");
}

bool c_seed_fill_segmentation_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, threshold);
    return true;
  }
  return false;
}

bool c_seed_fill_segmentation_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat1i labels;

  seed_fill_segmentation(image.getMat(), labels, _threshold);
  image.move(labels);
  return true;
}

