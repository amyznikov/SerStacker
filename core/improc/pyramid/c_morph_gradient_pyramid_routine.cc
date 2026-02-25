/*
 * c_morph_gradient_pyramid_routine.cc
 *
 *  Created on: Sep 3, 2023
 *      Author: amyznikov
 */

#include "c_morph_gradient_pyramid_routine.h"
#include <core/proc/morphology.h>

void c_morph_gradient_pyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "operation", ctx(&this_class::_operation), "");
  ctlbind(ctls, "max_level", ctx(&this_class::_max_level), "");
  ctlbind(ctls, "display_pos", ctx(&this_class::_display_pos), "");
}

bool c_morph_gradient_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _operation);
    SERIALIZE_OPTION(settings, save, *this, _max_level);
    SERIALIZE_OPTION(settings, save, *this, _display_pos);
    return true;
  }
  return false;
}

bool c_morph_gradient_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int display_pos = -1;

  if( image.needed() && !image.empty() ) {

    build_morph_gradient_pyramid(image,
        _ignore_mask ? cv::noArray() : mask,
        _pyramid,
        _max_level,
        _operation);

    display_pos =
        std::max(0, std::min(_display_pos,
            (int) _pyramid.size() - 1));

    _pyramid[display_pos].copyTo(image);
  }

  if( mask.needed() && !mask.empty() && display_pos > 0 ) {
    cv::resize(mask.getMat(), mask, _pyramid[display_pos].size(), 0, 0, cv::INTER_AREA);
    cv::compare(mask, 250, mask, cv::CMP_GE);
  }

  return true;
}
