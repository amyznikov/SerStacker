/*
 * c_stereo_rectification_routine.cc
 *
 *  Created on: Mar 23, 2023
 *      Author: amyznikov
 */

#include "c_stereo_rectification_routine.h"

template<>
const c_enum_member* members_of<c_stereo_rectification_routine::OverlayMode>()
{
  static constexpr c_enum_member members[] = {
      { c_stereo_rectification_routine::OverlayNone, "None", "No overlay" },
      { c_stereo_rectification_routine::OverlayAdd, "Add", "cv::addWeighted(left, right)" },
      { c_stereo_rectification_routine::OverlayAbsdiff, "Absdiff", "cv::absdiff(left, right)" },
      { c_stereo_rectification_routine::OverlayNone },
  };

  return members;
}
