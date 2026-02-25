/*
 * c_laplacian_pyramid_routine.cc
 *
 *  Created on: May 30, 2023
 *      Author: amyznikov
 */

#include "c_laplacian_pyramid_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_laplacian_pyramid_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_laplacian_pyramid_routine::DisplayLaplacian, "Laplacian", "Display Laplacian" },
      { c_laplacian_pyramid_routine::DisplayMean, "Mean", "Display Mean" },
      { c_laplacian_pyramid_routine::DisplayLaplacian },
  };

  return members;
}

bool c_laplacian_pyramid_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display_type);
    SERIALIZE_OPTION(settings, save, *this, _display_level);
    SERIALIZE_OPTION(settings, save, *this, _absdiff);
    SERIALIZE_OPTION(settings, save, *this, _minimum_image_size);
    return true;
  }
  return false;
}

void c_laplacian_pyramid_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "display_type",  ctx(&this_class::_display_type), "");
   ctlbind(ctls, "borderType",  ctx(&this_class::_borderType), "");
   ctlbind(ctls, "minimum_image_size",  ctx(&this_class::_minimum_image_size), "");
   ctlbind(ctls, "display_level",  ctx(&this_class::_display_level), "");
   ctlbind(ctls, "absdiff",  ctx(&this_class::_absdiff), "");
}

bool c_laplacian_pyramid_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  build_laplacian_pyramid(image,
      lp_,
      std::max(1, _minimum_image_size),
      _borderType,
      &mp_);

  if( lp_.size() < 1 ) {
    CF_ERROR("build_laplacian_pyramid() fails");
    return false;
  }

  switch (_display_type) {
    case DisplayMean: {
      const int display_level =
          std::max(0, std::min(_display_level,
              (int) mp_.size() - 1));

      mp_[display_level].copyTo(image);
      break;
    }
    case DisplayLaplacian:
      default: {
      const int display_level =
          std::max(0, std::min(_display_level,
              (int) lp_.size() - 1));

      if ( _absdiff ) {
        cv::absdiff(lp_[display_level], cv::Scalar::all(0), image);
      }
      else {
        lp_[display_level].copyTo(image);
      }
      break;
    }
  }


  if ( mask.needed() && !mask.empty() ) {
    mask.release();
  }

  return true;
}

