/*
 * c_downstrike_routine.cc
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#include "c_downstrike_routine.h"

template<>
const c_enum_member * members_of<c_downstrike_routine::DownstrikeMode>()
{
  static constexpr c_enum_member members[] = {
      {c_downstrike_routine::DownstrikeUneven, "Uneven"},
      {c_downstrike_routine::DownstrikeEven, "Even"},
      {c_downstrike_routine::DownstrikeUneven},
  };

  return members;
}
