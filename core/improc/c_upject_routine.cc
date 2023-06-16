/*
 * c_upject_routine.cc
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#include "c_upject_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_upject_routine::UpjectMode>()
{
  static constexpr c_enum_member members[] = {
      {c_upject_routine::UpjectUneven, "Uneven"},
      {c_upject_routine::UpjectEven, "Even"},
      {c_upject_routine::UpjectUneven},
  };

  return members;
}

template<>
const c_enum_member * members_of<c_upject_routine::FillMode>()
{
  static constexpr c_enum_member members[] = {
      {c_upject_routine::FillZero, "Zeros"},
      {c_upject_routine::FillAvg, "Avg"},
      {c_upject_routine::FillZero},
  };

  return members;
}
