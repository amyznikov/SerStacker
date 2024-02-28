/*
 * c_hdl_processor_routine.cc
 *
 *  Created on: Feb 28, 2024
 *      Author: amyznikov
 */

#include "c_hdl_processor_routine.h"


bool c_hdl_processor_routine::process(c_data_frame::sptr & dataframe)
{
  c_hdl_data_frame * hdl =
      dynamic_cast<c_hdl_data_frame*>(dataframe.get());

  if( !hdl ) {
    CF_ERROR("c_hdl_processor_routine: \n"
        "dynamic_cast<c_hdl_frame*>() fails");
    return false;
  }

  return process(hdl);
}

