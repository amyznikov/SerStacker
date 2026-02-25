/*
 * c_melp_pyramid_routine.cc
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#include "c_melp_pyramid_routine.h"
#include <core/ssprintf.h>

void c_melp_pyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "minimum_image_size",  ctx(&this_class::_minimum_image_size), "");
   ctlbind(ctls, "display_pos",  ctx(&this_class::_display_pos), "");
}

bool c_melp_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _minimum_image_size);
    SERIALIZE_OPTION(settings, save, *this, _display_pos);
    return true;
  }
  return false;
}

bool c_melp_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int minimum_image_size = 4;

  _pyramid = build_melp_pyramid(image, std::max(4, _minimum_image_size));

  static const auto recurse =
      [](c_melp_pyramid::sptr p, int h, int v) -> c_melp_pyramid::sptr {

        for ( int i = 0; i < h && p; ++i ) {
          p = p->l;
        }
        for ( int i = 0; i < v && p; ++i ) {
          p = p->m;
        }

        return p;
      };


  c_melp_pyramid::sptr p = _pyramid;

  CF_DEBUG("display_pos_.size()=%zu", _display_pos.size());

  for( int i = 0, n = _display_pos.size(); i < n && p; i += 2 ) {
    const int h = _display_pos[i];
    const int v = i < n - 1 ? _display_pos[i + 1] : 0;

    p = recurse(p, h, v);
  }


  if ( p ) {
    p->image.copyTo(image);
  }
  else {
    image.release();
  }

  if( mask.needed() && !mask.empty() ) {
    mask.release();
  }

  return true;
}
