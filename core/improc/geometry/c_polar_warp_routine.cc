/*
 * c_polar_warp_routine.cc
 *
 *  Created on: Aug 19, 2023
 *      Author: amyznikov
 */

#include "c_polar_warp_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_polar_warp_routine::INTERPOLATION_MODE>()
{
  static constexpr c_enum_member members[] = {
      {c_polar_warp_routine::INTER_LINEAR, "LINEAR", "cv::INTER_LINEAR"},
      {c_polar_warp_routine::INTER_AREA, "AREA", "cv::INTER_AREA"},
      {c_polar_warp_routine::INTER_CUBIC, "CUBIC", "cv::INTER_CUBIC"},
      {c_polar_warp_routine::INTER_LANCZOS4, "LANCZOS4", "cv::INTER_LANCZOS4"},
      {c_polar_warp_routine::INTER_LINEAR_EXACT, "LINEAR_EXACT", "cv::INTER_LINEAR_EXACT"},
      {c_polar_warp_routine::INTER_NEAREST, "NEAREST", "cv::INTER_NEAREST"},
#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0) )
    {c_polar_warp_routine::INTER_NEAREST_EXACT, "NEAREST_EXACT", "cv::INTER_NEAREST_EXACT"},
#endif
      {c_polar_warp_routine::INTER_LINEAR},
  };

  return members;
}

template<>
const c_enum_member * members_of<c_polar_warp_routine::WARP_MODE>()
{
  static constexpr c_enum_member members[] = {
      {c_polar_warp_routine::WARP_POLAR_LINEAR, "LINEAR", "cv::WARP_POLAR_LINEAR"},
      {c_polar_warp_routine::WARP_POLAR_LOG , "LOG", "cv::WARP_POLAR_LOG "},
      {c_polar_warp_routine::WARP_POLAR_LINEAR},
  };

  return members;
}
