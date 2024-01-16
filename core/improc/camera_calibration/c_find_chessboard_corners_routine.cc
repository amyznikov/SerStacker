/*
 * c_find_chessboard_corners_routine.cc
 *
 *  Created on: Feb 25, 2023
 *      Author: amyznikov
 */

#include "c_find_chessboard_corners_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_find_chessboard_corners_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_find_chessboard_corners_routine::DisplayCorners, "Corners", "" },
      { c_find_chessboard_corners_routine::DisplayOtsuImage, "OtsuImage", "" },
      { c_find_chessboard_corners_routine::DisplayHarrisImage, "HarrisImage", "" },
      { c_find_chessboard_corners_routine::DisplayHarrisThresholdImage, "HarrisThresholdImage", "" },
      { c_find_chessboard_corners_routine::DisplayHHarrisImage, "HHarrisImage", "" },
      { c_find_chessboard_corners_routine::DisplayVHarrisImage, "VHarrisImage", "" },
      { c_find_chessboard_corners_routine::DisplayCorners }
  };

  return members;
}

