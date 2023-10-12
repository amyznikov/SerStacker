/*
 * c_morph_gradient_pyramid_routine.cc
 *
 *  Created on: Sep 3, 2023
 *      Author: amyznikov
 */

#include "c_morph_gradient_pyramid_routine.h"
#include <core/proc/morphology.h>



void c_morph_gradient_pyramid_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  base::get_parameters(ctls);
  ADD_IMAGE_PROCESSOR_CTRL(ctls, max_level, "Specify minimum image size");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, display_pos, "Specify display node position");
}

bool c_morph_gradient_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, max_level);
    SERIALIZE_PROPERTY(settings, save, *this, display_pos);
    return true;
  }
  return false;
}

bool c_morph_gradient_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int display_pos = -1;

  if ( image.needed() && !image.empty() ) {

    build_morph_gradient_pyramid(image,
        ignore_mask_ ? cv::noArray() : mask,
        pyramid_,
        max_level_);

    display_pos =
        std::max(0, std::min(display_pos_,
            (int) pyramid_.size() - 1));

    pyramid_[display_pos].copyTo(image);
  }

  if ( mask.needed() && !mask.empty() && display_pos > 0 ) {
    cv::resize(mask.getMat(), mask, pyramid_[display_pos].size(), 0, 0, cv::INTER_AREA);
    cv::compare(mask, 250, mask, cv::CMP_GE);
  }

  return true;
}
