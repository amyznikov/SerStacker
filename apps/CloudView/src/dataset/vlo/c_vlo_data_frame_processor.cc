/*
 * c_vlo_data_frame_processor.cc
 *
 *  Created on: Dec 9, 2023
 *      Author: amyznikov
 */

#include "c_vlo_data_frame_processor.h"
#include <core/debug.h>

namespace cloudview {

c_vlo_data_frame_processor::c_vlo_data_frame_processor()
{
}


bool c_vlo_data_frame_processor::process(const c_cloudview_data_frame::sptr & dataframe)
{
  if ( c_vlo_data_frame * vlo = dynamic_cast<c_vlo_data_frame * >(dataframe.get()) ) {
    return process(vlo);
  }

  CF_ERROR("c_vlo_data_frame_processor: Can not cast input argument to c_vlo_data_frame");
  return false;
}

bool c_vlo_data_frame_processor::process(c_vlo_data_frame * vlo)
{
  CF_ERROR("Abstract function not implemented");
  return false;
}

} /* namespace cloudview */
