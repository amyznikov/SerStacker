/*
 * c_video_frame_processor_routine.cc
 *
 *  Created on: Jan 7, 2024
 *      Author: amyznikov
 */

#include "c_video_frame_processor_routine.h"


bool c_video_frame_processor_routine::process(c_data_frame::sptr & dataframe)
{
  c_video_frame * frame = dynamic_cast<c_video_frame*>(dataframe.get());

  if( !frame ) {
    CF_ERROR("c_video_frame_processor_routine: \n"
        "dynamic_cast<c_video_frame*>() fails");
    return false;
  }

  return process(frame);
}

