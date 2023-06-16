/*
 * c_gaussian_filter_routine.cc
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#include "c_gaussian_blur_routine.h"


template<>
const c_enum_member* members_of<c_gaussian_blur_routine::StereoMode>()
{
  static constexpr c_enum_member members[] = {
      { c_gaussian_blur_routine::StereoNone, "None", "Not at stereo (single frame)" },
      { c_gaussian_blur_routine::StereoHLayout, "HLayout", "Horizontal stereo frame" },
      { c_gaussian_blur_routine::StereoVLayout, "VLayout", "Vertical stereo frame" },
      { c_gaussian_blur_routine::StereoNone },
  };

  return members;
}
