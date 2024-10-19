/*
 * c_locate_extremes_routine.cc
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#include "c_local_minmax_routine.h"

#include <core/proc/reduce_channels.h>

template<>
const c_enum_member* members_of<c_local_minmax_routine::OUTPUT_CHANNEL>()
{
  static const c_enum_member members[] = {
      { c_local_minmax_routine::OUTPUT_IMAGE, "IMAGE", "" },
      { c_local_minmax_routine::OUTPUT_MASK, "MASK", "" },
      { c_local_minmax_routine::OUTPUT_MASK },
  };

  return members;
}

template<>
const c_enum_member* members_of<c_local_minmax_routine::BorderType>()
{
  static const c_enum_member members[] = {
      { c_local_minmax_routine::BORDER_CONSTANT, "BORDER_CONSTANT", "Set to BorderValue"},
      { c_local_minmax_routine::BORDER_REPLICATE, "BORDER_REPLICATE", "aaaaaa|abcdefgh|hhhhhhh"},
      { c_local_minmax_routine::BORDER_REFLECT, "BORDER_REFLECT", "fedcba|abcdefgh|hgfedcb"},
      { c_local_minmax_routine::BORDER_REFLECT_101, "BORDER_REFLECT_101", "gfedcb|abcdefgh|gfedcba"},
      { c_local_minmax_routine::BORDER_TRANSPARENT, "BORDER_TRANSPARENT", "uvwxyz|abcdefgh|ijklmno"},
      { c_local_minmax_routine::BORDER_REPLICATE},
  };

  return members;
}


void c_local_minmax_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, filter_type, "filter_type");

  BIND_PCTRL(ctls, se_shape, "se_shape");
  BIND_PCTRL(ctls, se_size, "se_size");
  BIND_PCTRL(ctls, anchor, "anchor");

  BIND_PCTRL(ctls, locate_maximums, "locate_maximums");
  BIND_PCTRL(ctls, maximums_alpha, "maximums_alpha");
  BIND_PCTRL(ctls, maximums_beta, "maximums_beta");

  BIND_PCTRL(ctls, locate_minimums, "locate_minimums");
  BIND_PCTRL(ctls, minimums_alpha, "minimums_alpha");
  BIND_PCTRL(ctls, minimums_beta, "minimums_beta");

  BIND_PCTRL(ctls, border_type, "border_type");
  BIND_PCTRL(ctls, border_value, "border_value");

  BIND_PCTRL(ctls, output_channel, "output_channel");
  BIND_PCTRL(ctls, ignore_mask, "ignore_mask");
}

bool c_local_minmax_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, filter_type);

    SERIALIZE_PROPERTY(settings, save, *this, se_shape);

    SERIALIZE_PROPERTY(settings, save, *this, se_size);
    SERIALIZE_PROPERTY(settings, save, *this, anchor);

    SERIALIZE_PROPERTY(settings, save, *this, locate_maximums);
    SERIALIZE_PROPERTY(settings, save, *this, maximums_alpha);
    SERIALIZE_PROPERTY(settings, save, *this, maximums_beta);

    SERIALIZE_PROPERTY(settings, save, *this, locate_minimums);
    SERIALIZE_PROPERTY(settings, save, *this, minimums_alpha);
    SERIALIZE_PROPERTY(settings, save, *this, minimums_beta);

    SERIALIZE_PROPERTY(settings, save, *this, border_type);
    SERIALIZE_PROPERTY(settings, save, *this, border_value);

    SERIALIZE_PROPERTY(settings, save, *this, output_channel);
    return true;
  }
  return false;
}

bool c_local_minmax_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat M;

  locate_extremes(image,
      _ignore_mask ? cv::noArray() :
          mask.getMat(),
      M,
      opts_);

  if ( output_channel_ == OUTPUT_IMAGE ) {
    image.move(M);
  }
  else if ( M.channels() > 1 ) {
    reduce_color_channels(M, mask, cv::REDUCE_MAX);
  }
  else {
    mask.move(M);
  }

  return true;
}
