/*
 * c_melp_pyramid_routine.cc
 *
 *  Created on: May 31, 2023
 *      Author: amyznikov
 */

#include "c_melp_pyramid_routine.h"

void c_melp_pyramid_routine::get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls)
{
  ADD_IMAGE_PROCESSOR_CTRL(ctls, min_image_size, "Specify minimum image size");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, display_row, "Specify display pyramid layer row index");
  ADD_IMAGE_PROCESSOR_CTRL(ctls, display_col, "Specify display pyramid layer col index");
}

bool c_melp_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, min_image_size);
    SERIALIZE_PROPERTY(settings, save, *this, display_row);
    SERIALIZE_PROPERTY(settings, save, *this, display_col);
    return true;
  }
  return false;
}

bool c_melp_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  int minimum_image_size = 4;

  build_melp_pyramid(image,
      &pyramid_,
      std::max(1, minimum_image_size_));

  if( pyramid_.l.size() < 1 ) {
    CF_ERROR("build_melp_pyramid() fails");
    return false;
  }

  const c_melp_pyramid *p = &pyramid_;
  for( int i = 0; i < display_row_ && !p->p.empty(); ++i ) {
    p = &p->p.front();
  }

  CF_DEBUG("p->l.size=%zu", p->l.size());

  const int display_col =
      std::max(0, std::min(display_col_,
          (int) p->l.size() - 1));

  p->l[display_col].copyTo(image);

  if( mask.needed() && !mask.empty() ) {
    mask.release();
  }

  return true;
}
