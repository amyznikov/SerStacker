/*
 * c_median_pyramid_routine.cc
 *
 *  Created on: Sep 10, 2023
 *      Author: amyznikov
 */

#include "c_median_pyramid_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_median_pyramid_routine::DisplayType>()
{
  static constexpr c_enum_member members[] = {
      { c_median_pyramid_routine::DisplayMedianBlur, "MedianBlur", "Display MedianBlur" },
      { c_median_pyramid_routine::DisplayMedianHat, "MedianHat", "Display MedianHat" },
      { c_median_pyramid_routine::DisplayMedianBlur },
  };

  return members;
}
