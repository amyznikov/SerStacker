/*
 * c_polar_warp_routine.cc
 *
 *  Created on: Aug 19, 2023
 *      Author: amyznikov
 */

#include "c_polar_warp_routine.h"
#include <core/proc/polar_trasform.h>
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_polar_warp_routine::INTERPOLATION_MODE>()
{
  static constexpr c_enum_member members[] = {
      { c_polar_warp_routine::INTER_LINEAR, "LINEAR", "cv::INTER_LINEAR" },
      { c_polar_warp_routine::INTER_AREA, "AREA", "cv::INTER_AREA" },
      { c_polar_warp_routine::INTER_CUBIC, "CUBIC", "cv::INTER_CUBIC" },
      { c_polar_warp_routine::INTER_LANCZOS4, "LANCZOS4", "cv::INTER_LANCZOS4" },
      { c_polar_warp_routine::INTER_LINEAR_EXACT, "LINEAR_EXACT", "cv::INTER_LINEAR_EXACT" },
      { c_polar_warp_routine::INTER_NEAREST, "NEAREST", "cv::INTER_NEAREST" },
      #if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0) )
      { c_polar_warp_routine::INTER_NEAREST_EXACT, "NEAREST_EXACT", "cv::INTER_NEAREST_EXACT" },
      #endif
      { c_polar_warp_routine::INTER_LINEAR },
  };

  return members;
}
//
//template<>
//const c_enum_member* members_of<c_polar_warp_routine::WARP_MODE>()
//{
//  static constexpr c_enum_member members[] = {
//      { c_polar_warp_routine::WARP_POLAR_LINEAR, "LINEAR", "cv::WARP_POLAR_LINEAR" },
//      { c_polar_warp_routine::WARP_POLAR_LOG, "LOG", "cv::WARP_POLAR_LOG " },
//      { c_polar_warp_routine::WARP_POLAR_LINEAR },
//  };
//
//  return members;
//}

#if 1
bool c_polar_warp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() || !mask.empty() ) {

    const cv::Size src_size =
        mask.empty() ? image.size() :
            mask.size();

    if( rmap_.empty() || old_src_size_ != src_size ) {
      old_src_size_ = src_size;
      create_epipolar_remap(src_size, center_, rmap_);
    }

    if( !image.empty() ) {
      cv::remap(image, image, rmap_, cv::noArray(),
          interpolation_mode_,
          cv::BORDER_CONSTANT);
    }

    if( mask.needed() ) {
      if( !mask.empty() ) {
        cv::remap(mask, mask, rmap_, cv::noArray(),
            interpolation_mode_,
            cv::BORDER_CONSTANT);
      }
      else {
        cv::remap(cv::Mat1b(src_size, 255), mask, rmap_, cv::noArray(),
            interpolation_mode_,
            cv::BORDER_CONSTANT);
      }
      cv::compare(mask, 254, mask, cv::CMP_LT);
    }
  }

  return true;
}

#else

bool c_polar_warp_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() || !mask.empty()) {

    const cv::Size src_size =
        mask.empty() ? image.size() :
            mask.size();

    cv::Mat1b msk;
    cv::warpPolar(cv::Mat1b(src_size, (uint8_t)255), msk,
        dsize_,
        center_,
        maxRadius_,
        cv::INTER_LINEAR | warp_mode_);
    cv::compare(msk, 254, msk, cv::CMP_LT);


    if( !image.empty() ) {
      cv::warpPolar(image.getMat(), image,
          dsize_,
          center_,
          maxRadius_,
          interpolation_mode_ | warp_mode_);
      image.setTo(0, msk);
    }

    if ( mask.needed() ) {
      if( mask.empty() ) {
        cv::bitwise_not(msk, mask);
      }
      else {
        cv::warpPolar(mask.getMat(), mask,
            dsize_,
            center_,
            maxRadius_,
            cv::INTER_LINEAR | warp_mode_);
        cv::compare(mask, 254, mask, cv::CMP_GE);
        mask.setTo(0, msk);
      }
    }
  }

  return true;
}
#endif
