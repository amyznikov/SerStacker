/*
 * c_laplacian_pyramid_routine.cc
 *
 *  Created on: May 30, 2023
 *      Author: amyznikov
 */

#include "c_laplacian_pyramid_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<c_laplacian_pyramid_routine::DisplayType>()
{
  static const c_enum_member members[] = {
      { c_laplacian_pyramid_routine::DisplayLaplacian, "Laplacian", "Display Laplacian" },
      { c_laplacian_pyramid_routine::DisplayMean, "Mean", "Display Mean" },
      { c_laplacian_pyramid_routine::DisplayLaplacian },
  };

  return members;
}
