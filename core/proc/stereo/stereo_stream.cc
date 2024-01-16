/*
 * stereo_stream.cc
 *
 *  Created on: Mar 18, 2023
 *      Author: amyznikov
 */

#include "stereo_stream.h"
#include <core/ssprintf.h>

template<>
const c_enum_member* members_of<stereo_stream_layout_type>()
{
  static const c_enum_member members[] = {
      { stereo_stream_layout_horizontal_split, "horizontal", "left and right frames merged horizontally" },
      { stereo_stream_layout_vertical_split, "vertical", "left and right frames merged vertically" },
      //{ stereo_stream_layout_interleaved_frames, "interleaved", "interleave odd left frame and even right frame" },
      { stereo_stream_layout_horizontal_split }
  };

  return members;
}
