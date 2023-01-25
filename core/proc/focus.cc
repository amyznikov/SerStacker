/*
 * focus.cc
 *
 *  Created on: Jan 8, 2023
 *      Author: amyznikov
 */
#include "focus.h"
#include <core/ssprintf.h>
#include <core/debug.h>


template<>
const c_enum_member* members_of<SHARPNESS_MEASURE>()
{
  static constexpr c_enum_member members[] = {
      { SHARPNESS_MEASURE_LCM, "LCM", "c_local_contrast_measure" },
      { SHARPNESS_MEASURE_LPG, "LPG", "c_lpg_sharpness_measure" },
      { SHARPNESS_MEASURE_LPG },
  };

  return members;
}
