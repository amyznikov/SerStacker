/*
 * c_melp_pyramid_routine.cc
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#include "c_melp_pyramid_routine.h"
#include <core/ssprintf.h>

void c_melp_pyramid_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, min_image_size, "Specify minimum image size");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, display_pos, "Specify display node position");
}

bool c_melp_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, min_image_size);
    SERIALIZE_PROPERTY(settings, save, *this, display_pos);
    return true;
  }
  return false;
}

bool c_melp_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int minimum_image_size = 4;

  pyramid_ = build_melp_pyramid(image, std::max(4, minimum_image_size_));

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

  c_melp_pyramid::sptr p = pyramid_;

  CF_DEBUG("display_pos_.size()=%zu", display_pos_.size());

  for( int i = 0, n = display_pos_.size(); i < n && p; i += 2 ) {
    const int h = display_pos_[i];
    const int v = i < n - 1 ? display_pos_[i + 1] : 0;

    CF_DEBUG("recurse to h=%d v=%d", h, v);

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
