/*
 * c_ego_motion_correction.cc
 *
 *  Created on: Oct 10, 2018
 *      Author: amyznikov
 */

#include "c_ego_motion_compensation.h"
#include <core/ssprintf.h>

#ifdef _MSC_VER
# define strcasecmp(a, b) 	_stricmp(a, b)
#endif 


template<>
const c_enum_member * members_of<TRAJECTORY_ESTIMATION_METHOD>()
{
  static const c_enum_member members[] = {

      {TRAJECTORY_ESTIMATION_GPS, "GPS",
          "Uses only gps positions lat, lon, alt, and orientation roll, pitch, yaw for trajectory estimation"
      },

      {TRAJECTORY_ESTIMATION_IMU, "IMU",
          "Uses only velocities vf, vl, vu  and orientation roll, pitch, yaw for trajectory estimation,\n"
          "simplified linear motion approximation (does not account the changes in roll, pitch, yaw during  LIDAR frame acquisition)"
      },

      {TRAJECTORY_ESTIMATION_IMUE, "IMUE",
          "Euler integration of vf, vl, vu  and orientation roll, pitch, yaw for trajectory estimation,\n"
          "Uses oxts[] measurements within single LIDAR frame if available for more precise trajectory estimation.\n"
          "Note that the Norway trace example does not expose strong yaws, therefore resulting trajectory\n"
          "is mostly identical to one build from simplified linear approximation TRAJECTORY_ESTIMATION_IMU.\n"
      },

      {TRAJECTORY_ESTIMATION_NONE, "NONE",
          "Stub for silently skip ego motion compensation"
      },

      {TRAJECTORY_ESTIMATION_NONE},
  };

  return members;
}

