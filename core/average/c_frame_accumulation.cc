/*
 * c_frame_accumulation.cc
 *
 *  Created on: Feb 14, 2021
 *      Author: amyznikov
 */

#include "c_frame_accumulation.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<DRIZZLE_SCALE>()
{
  static const c_enum_member members[] = {
      { DRIZZLE_SCALE_10x, "DRIZZLE_x1.0", "" },
      { DRIZZLE_SCALE_15x, "DRIZZLE_x1.5", "" },
      { DRIZZLE_SCALE_20x, "DRIZZLE_x2.0", "" },
      { DRIZZLE_SCALE_30x, "DRIZZLE_x3.0", "" },
      { DRIZZLE_SCALE_40x, "DRIZZLE_x4.0", "" },
      { DRIZZLE_SCALE_10x}
  };
  return members;
}

