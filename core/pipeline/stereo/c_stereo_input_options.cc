/*
 * c_stereo_input_options.cc
 *
 *  Created on: Jul 17, 2023
 *      Author: amyznikov
 */

#include "c_stereo_input_options.h"


bool serialize_base_stereo_input_options(c_config_setting section, bool save, c_stereo_input_options & opts)
{
  SERIALIZE_OPTION(section, save, opts.input_source, layout_type);
  SERIALIZE_OPTION(section, save, opts.input_source, swap_cameras);
  SERIALIZE_OPTION(section, save, opts.input_source, left_stereo_source);
  SERIALIZE_OPTION(section, save, opts.input_source, right_stereo_source);

  serialize_base_input_options(section, save, opts);

  return true;
}
