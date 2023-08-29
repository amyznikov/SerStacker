/*
 * c_median_hat_routine.cc
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#include "c_median_hat_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_median_hat_routine::DisplayType>()
{
  static constexpr c_enum_member members[] =  {
      {c_median_hat_routine::DisplayMedianBlur, "MedianBlur", "MedianBlur"} ,
      {c_median_hat_routine::DisplayMedianHat, "MedianHat", "Difference between source image and its median blur" },
      {c_median_hat_routine::DisplayMedianHat },
  };

  return members;
}
