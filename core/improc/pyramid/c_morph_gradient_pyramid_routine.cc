/*
 * c_morph_gradient_pyramid_routine.cc
 *
 *  Created on: Sep 3, 2023
 *      Author: amyznikov
 */

#include "c_morph_gradient_pyramid_routine.h"
#include <core/proc/morphology.h>


void c_morph_gradient_pyramid_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  base::get_parameters(ctls);
  BIND_SPINBOX_CTRL(ctls, max_level, 0, 32, 1,  "max_level", "Specify max pyramid level");
  BIND_SPINBOX_CTRL(ctls, display_pos, 0, 32, 1, "display level", "Specify display pyramid level");
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
        _ignore_mask ? cv::noArray() : mask,
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
