/*
 * c_vlo_processor_routine.cc
 *
 *  Created on: Dec 16, 2023
 *      Author: amyznikov
 */

#include "c_vlo_processor_routine.h"
#include <core/debug.h>

bool c_vlo_processor_routine::process(c_data_frame::sptr & dataframe)
{
  c_vlo_data_frame * vlo =
      dynamic_cast<c_vlo_data_frame*>(dataframe.get());

  if( !vlo ) {
    CF_ERROR("c_vlo_processor_routine: \n"
        "dynamic_cast<c_vlo_frame*>() fails");
    return false;
  }

  return process(vlo);
}

