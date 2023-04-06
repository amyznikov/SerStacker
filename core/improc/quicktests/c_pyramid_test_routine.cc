/*
 * c_pyramid_test_routine.cc
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#include "c_pyramid_test_routine.h"
#include <core/proc/downstrike.h>

template<>
const c_enum_member * members_of<c_pyramid_test_routine::Mode>()
{
  static constexpr c_enum_member members[] = {
    {c_pyramid_test_routine::TestPyrDown, "PyrDown", ""},
    {c_pyramid_test_routine::TestPyrUp, "PyrUp", ""},
    {c_pyramid_test_routine::TestPyrDownUp, "PyrDownUp", ""},
    {c_pyramid_test_routine::TestPyrDownUp}
  };

  return members;
}


bool c_pyramid_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{

  switch (mode_) {
    case TestPyrDown:
      pyrdown(image.getMat(), image);
      break;
    case TestPyrUp:
      pyrup(image.getMat(), image);
      break;
    case TestPyrDownUp:
      pyrdown(image.getMat(), image);
      pyrup(image.getMat(), image);
      break;
  }

  if( mask.needed() && !mask.empty() && mask.size() != image.size() ) {
    cv::resize(mask.getMat(), mask, image.size(), 0, 0, cv::INTER_NEAREST);
  }

  return true;
}

void c_pyramid_test_routine::pyrdown(cv::InputArray src, cv::OutputArray dst,
    const cv::Size & dstsize,
    int borderType)
{

}

void c_pyramid_test_routine::pyrup(cv::InputArray src, cv::OutputArray dst,
    const cv::Size & dstsize,
    int borderType)
{

}

