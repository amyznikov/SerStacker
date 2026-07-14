/*
 * configpath.cc
 *
 *  Created on: Jul 15, 2026
 *      Author: amyznikov
 */

#include "configpath.h"
#include <core/readdir.h>

const std::string & get_default_config_path()
{
  static const std::string config_path = expand_path("~/.config/SerStacker/");
  return config_path;
}

